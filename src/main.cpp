#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESPTelnet.h>
#include <ESPmDNS.h>
#include <TMCStepper.h>
#include <WebServer.h>
#include <WiFi.h>
// #include <Adafruit_NeoPixel.h>

#include <ArduinoJson.h>
#include <LittleFS.h>
#include <WebSocketsServer.h>

#if (defined(CONFIG_IDF_TARGET_ESP32C3) || defined(ARDUINO_ESP32C3_DEV))
// Alternativa: macro que ignora o parâmetro core
#define xTaskCreatePinnedToCore(task, name, stack, param, prio, handle, core)  \
  xTaskCreate(task, name, stack, param, prio, handle)
#endif

#include <Preferences.h>
#include <Wire.h>

#if (defined(CONFIG_IDF_TARGET_ESP32C3) || defined(ARDUINO_ESP32C3_DEV))
// ESP32-ESP32S2-AnalogWrite
//  #include <pwmWrite.h>
// Pwm oServo = Pwm();
#include <ESP32Servo.h>
Servo oServo;

#include <MT6701.h>
#elif (defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV))
#include <ESP32ServoController.h>
using namespace MDO::ESP32ServoController;
ServoController oServo;

#include "MT6701.hpp"

#elif 0
#include <Servo.h>;
Servo oServo;

#include <MT6701.h>
#endif

Preferences prefs;

ESPTelnet telnet;
MT6701 tonearm;

#define DEBUG_PRINT(x)                                                         \
  do {                                                                         \
    telnet.println(x);                                                         \
    Serial.println(x);                                                         \
} while (0)
#define DEBUG_PRINTF(...)                                                      \
  do {                                                                         \
    telnet.printf(__VA_ARGS__);                                                \
    Serial.printf(__VA_ARGS__);                                                \
} while (0)

// Defina o pino e número de LEDs (geralmente 1 no onboard)
#define LED_PIN 8 // Tente 48 primeiro (mais comum no N8)
// #define LED_PIN 38    // Se não funcionar com 48, teste 38 (algumas revisões
// v1.1)
#define NUM_LEDS 1

  // Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

  bool wifiConectadoAnterior = false;

// Velocidade desejada (RPM) - você pode mudar via Serial
volatile float targetRPM = 33.33; // ← ALTERE AQUI ou via Serial Monitor
float currentRPM = 0.0;
float targetRPMSet = 0.0;
unsigned long lastRampUpdate = 0;
const unsigned long RAMP_INTERVAL_MS = 20; // atualiza a cada 20 ms
float rampIncrementPerStep = 0;            // calculado quando inicia rampa
bool isRamping = false;

static const int servoPin = 10;

static const int tonearmPin_SDA = 6;
static const int tonearmPin_SCL = 7;

bool motorLigado = false;     // Estado inicial
bool posicaoLift = true;      // Estado inicial
float posicaoLiftMin = 120.0; // baixado
float posicaoLiftMax = 80.0;  // levantado
bool finalDisco = false;

// Debounce para ligar motor ao baixar tonearm
unsigned long lowAngleStartTime = 0; // Timestamp quando ângulo primeiro <=160°
unsigned long DEBOUNCE_DELAY_MS = 1500; // 2 segundos
bool debounceLowAngleActive =
    false; // Flag para rastrear se estamos contando tempo

// STEPPER MOTOR

const float CLOCK_CORRECTION =
    1.01099f; // comece com esse valor e ajuste ±0.001 até bater exato
// Motor NEMA17 padrão (200 passos/volta)
#define FULL_STEPS 200
#define MICROSTEPS 256
// Pinos UART no ESP32 (half-duplex)
#define UART_RX_PIN 5
#define UART_TX_PIN 4

// Corrente RMS (ajuste conforme seu motor - comece baixo!)
#define RMS_CURRENT_MA 700 // Ex: 400-800mA para NEMA17 comum

// Pinos UART para o TMC2209
#define DRIVER_ADDRESS 0b00 // Endereço padrão
#define R_SENSE 0.11f       // Valor padrão para drivers StepStick

// Pinos
// const int pinoDirecao = 0;
// const int pinoPasso = 1;
const int pinoEnable = 2;

HardwareSerial mySerial(1);
TMC2209Stepper driver(&mySerial, R_SENSE, DRIVER_ADDRESS);

// Configuração Wifi
const char *ssid = "SALVATI";
const char *password = "2007112402";

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
unsigned long lastStatusUpdate = 0;

// TMC2209
// Variáveis de Controle
float rpmSelecionado = 33.333;
float ajusteFino33 = 1.0;
float ajusteFino45 = 1.0;
float posicaoServo = posicaoLiftMax;
bool atualizando = false;

void moveServo(float angle, unsigned long time, bool hold) {
// if (chipModel == "ESP32-C3") {
#if (defined(CONFIG_IDF_TARGET_ESP32C3) || defined(ARDUINO_ESP32C3_DEV))

  // Runtime logic for C3
  // oServo.write(posicaoServo, time);
  oServo.write(posicaoServo);
//} else if (chipModel == "ESP32-S3") {
#elif (defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV))
  // Runtime logic for S3
  oServo.moveTo(angle, time, hold);
//} else {
#elif 0
  // Handle other models
  oServo.write(servoPin, angle);
//}
#endif
}

void checkThermalStatus() {
  uint32_t drv_status = driver.DRV_STATUS();

  // otpw: Over Temperature Pre-Warning (~120°C)
  if (driver.otpw()) {
    DEBUG_PRINT("ALERTA: Driver atingiu 120°C! Considere reduzir a corrente.");
  }

  // ot: Over Temperature Critical (~150°C)
  if (driver.ot()) {
    DEBUG_PRINT("ERRO: Driver desligado por superaquecimento!");
  }
}

// ===================== FUNÇÃO DE RPM =====================
// Calcula VACTUAL corretamente (fCLK = 12 MHz interno)
void setRPM(float rpm) {
  if (rpm == 0) {
    driver.VACTUAL(0);
    digitalWrite(pinoEnable, HIGH);
    return;
  } else {
    digitalWrite(pinoEnable, LOW);
  }
  // rpm = rpm * 0.99;

  // Velocidade em microsteps/segundo
  float usteps_per_sec = (rpm / 60.0f) * FULL_STEPS * MICROSTEPS;

  // Fórmula oficial do datasheet TMC2209
  // VACTUAL = usteps/s * (2^24 / 12.000.000)
  // int32_t vactual = (int32_t)round(usteps_per_sec * (16777216.0f /
  // 12000000.0f));
  float factor = 16777216.0f / (12000000.0f * CLOCK_CORRECTION);
  int32_t vactual = (int32_t)round(usteps_per_sec * factor);

  driver.VACTUAL(vactual);
  DEBUG_PRINTF("VACTUAL = %ld  (RPM = %.1f)\n", vactual, rpm);
}

// Acelera gradualmente até a RPM alvo
// accelTimeMs = tempo total da rampa em milissegundos (ex: 2000 = 2 segundos)
// currentRPM = velocidade atual (guarde em uma variável global)
void accelerateTo(float targetRPM, unsigned long accelTimeMs = 1500) {
  if (targetRPM <= 0) {
    driver.VACTUAL(0);
    currentRPM = 0;
    return;
  }

  float startRPM = currentRPM; // começa de onde está
  if (startRPM < 0)
    startRPM = 0;

  unsigned long startTime = millis();
  unsigned long elapsed;

  do {
    elapsed = millis() - startTime;
    float progress = (float)elapsed / accelTimeMs;
    if (progress > 1.0)
      progress = 1.0;

    float nowRPM = startRPM + (targetRPM - startRPM) * progress;
    setRPM(nowRPM); // usa a função setRPM que já existe

    delay(10); // atualização a cada ~10 ms → suave o suficiente
  } while (elapsed < accelTimeMs);

  currentRPM = targetRPM; // atualiza a variável global
  setRPM(targetRPM);      // garante o valor final exato
}

void toggleMotor(bool ligar = false) {
  int8_t result = driver.test_connection();

  if (result == 0) {
    DEBUG_PRINT("SUCESSO: UART comunicando!");
    Serial.print("SUCESSO: UART comunicando!");
  } else {
    DEBUG_PRINTF("ERRO: Falha na comunicação (Código: %d)\n", result);
    Serial.printf("ERRO: Falha na comunicação (Código: %d)\n", result);
    DEBUG_PRINT("Verifique: 1. Alimentação VMOT (12V) ligada? 2. Resistor de "
                "1k? 3. Pinos TX/RX invertidos?");
    digitalWrite(pinoEnable, HIGH);
  }
  if (ligar) {
    // startRampTo(targetRPM,3);
    posicaoServo = posicaoLiftMin;
    // ledcDetachPin(servoPin);
    moveServo(posicaoServo, 1600, true);
    posicaoLift = false;

    // liga motor
    driver.rms_current(1000); // 1000mA (limite do seu motor)

    setRPM(targetRPM);

    // 2. Espera o prato vencer a inércia (ex: 2 segundos)
    // Nota: Em um código profissional, usaríamos um timer não bloqueante,
    // mas para teste inicial o delay resolve:
    delay(2000);

    // 3. Volta para a corrente de cruzeiro para não esquentar o motor
    driver.rms_current(RMS_CURRENT_MA); // Volta para os 800mA

  } else {
    posicaoServo = posicaoLiftMax;
    moveServo(posicaoServo, 400, false);
    posicaoLift = true;

    // desliga motor
    setRPM(0);
    // delay(4100);  // ou melhor: use um timer não bloqueante
    // ledcDetachPin(servoPin);
  }
  motorLigado = !motorLigado;
}

void startRampTo(float newTargetRPM, float accelTimeSeconds = 2.0) {
  targetRPMSet = newTargetRPM;

  if (accelTimeSeconds <= 0.05)
    accelTimeSeconds = 0.05;

  float deltaRPM = newTargetRPM - currentRPM;
  float steps = (accelTimeSeconds * 1000.0) / RAMP_INTERVAL_MS;

  rampIncrementPerStep = deltaRPM / steps;

  isRamping = true;
  lastRampUpdate = millis();
}

void broadcastStatus() {
  JsonDocument doc;
  doc["motorLigado"] = motorLigado;

#if (defined(CONFIG_IDF_TARGET_ESP32C3) || defined(ARDUINO_ESP32C3_DEV))
  doc["tonearmAngle"] = tonearm.angleRead();
#elif (defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV))
  doc["tonearmAngle"] = tonearm.getAngleDegrees();
#elif 0
  doc["tonearmAngle"] = tonearm.angleRead();
#endif

  doc["rpm"] = rpmSelecionado;
  doc["servoPos"] = posicaoServo;
  doc["liftMax"] = posicaoLiftMax;
  doc["liftMin"] = posicaoLiftMin;

  String jsonStr;
  serializeJson(doc, jsonStr);
  webSocket.broadcastTXT(jsonStr);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload,
                    size_t length) {
  if (type == WStype_TEXT) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (!error) {
      String cmd = doc["cmd"].as<String>();
      if (cmd == "toggle") {
        toggleMotor(!motorLigado);
      } else if (cmd == "rpm") {
        float r = doc["val"].as<float>();
        if (r < 40)
          rpmSelecionado = 33.333;
        else
          rpmSelecionado = 45.0;
        targetRPM = rpmSelecionado;
        setRPM(targetRPM);
      } else if (cmd == "servo") {
        float novaPos = doc["val"].as<float>();
        if (novaPos >= posicaoLiftMax && novaPos <= posicaoLiftMin) {
          posicaoServo = novaPos;
          moveServo(posicaoServo, 200, false);
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200); // use Serial normal do ESP32 para debug
  Serial.setTxTimeoutMs(0); // Evita que o ESP32 trave se o Serial Monitor estiver fechado
  DEBUG_PRINT("Setup init...");

#if (defined(CONFIG_IDF_TARGET_ESP32C3) || defined(ARDUINO_ESP32C3_DEV))
  // Runtime logic for C3
  oServo.attach(servoPin);
#elif (defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV)) && 0
  // Runtime logic for S3
  // configure our main settings in the ESP32 LEDC registry
  Esp32LedcRegistry::instance()->begin(
      LEDC_CONFIG_ESP32_S3); // change this for the relevant
  BestAvailableFactory
      oTimerChannelFactory; // used to select the best available timer & channel
                            // based on the hardware setup
  ServoFactoryDecorator oFactoryDecorator(
      oTimerChannelFactory); // let this ServoFactoryDecorator define the servo
                             // frequency to use and such
  // the above two are needed (variable scope related, in 'begin' only)

  // if (!oServo.begin(oFactoryDecorator, servoPin)) {
  // //3rd parameter is the default angle to start from: 90 degrees in this case
  // Serial.println("  failed to init the servo..");
  // pixels.setPixelColor(0, pixels.Color(255, 255, 0));
  // pixels.show();
  // return;
  //}
#elif 0
  // Handle other models
  oServo.attach(servoPin);
#endif

  DEBUG_PRINT("Servo...OK");

  // pinMode(pinoDirecao, OUTPUT);
  // pinMode(pinoPasso, OUTPUT);
  pinMode(pinoEnable, OUTPUT);

  // digitalWrite(pinoDirecao, LOW);
  digitalWrite(pinoEnable, HIGH); // deixar desligado
  // digitalWrite(pinoPasso, LOW);

  DEBUG_PRINT("Pins...OK");

  // pixels.begin();           // Inicializa o NeoPixel
  // pixels.setBrightness(50); // 0-255, comece baixo para não ofuscar (50 é
  // bom) pixels.clear();           // Apaga o LED no início
  // pixels.setPixelColor(0, pixels.Color(0, 0, 255));  // Blue
  // pixels.show();

  // Inicia comunicação UART com o Driver
  mySerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  driver.begin();
  delay(200);
  driver.toff(2); // Habilita o driver
  driver.rms_current(RMS_CURRENT_MA);
  // driver.mstep_reg_select(1);  // necessary for TMC2208 to set microstep
  // register with UART
  driver.microsteps(MICROSTEPS);
  driver.I_scale_analog(false); // Usa corrente via UART (não potenciômetro)

  driver.en_spreadCycle(
      false);                  // DESLIGA SpreadCycle (obrigatório para Stealth)
  driver.pwm_autoscale(false); // Ativa StealthChop2
  driver.pwm_freq(2);          // Auto-tuning do PWM (ainda mais silencioso)
  driver.pwm_ofs(150); // Valor de amplitude inicial (ajusta a força do campo
                       // magnético parado)
  driver.pwm_grad(4);  // Gradiente de aceleração do PWM
  driver.pwm_autograd(true); // Auto-tuning do PWM (ainda mais silencioso)

  driver.TPWMTHRS(0); // Fica 100% em StealthChop (sem troca de modo)

  DEBUG_PRINT("Driver...OK");
  // Ajustes finos para ruído em repouso
  // driver.iholddelay(10);          // Atraso para reduzir corrente em repouso
  // driver.TPOWERDOWN(128);         // Tempo para entrar em modo de economia
  // driver.pwm_temp_stepdown(true); // Proteção térmica que reduz ruído

  int8_t result = driver.test_connection();

  if (result == 0) {
    DEBUG_PRINT("SUCESSO: UART comunicando!");
    Serial.print("SUCESSO: UART comunicando!");
  } else {
    DEBUG_PRINTF("ERRO: Falha na comunicação (Código: %d)\n", result);
    Serial.printf("ERRO: Falha na comunicação (Código: %d)\n", result);
    DEBUG_PRINT("Verifique: 1. Alimentação VMOT (12V) ligada? 2. Resistor de "
                "1k? 3. Pinos TX/RX invertidos?");
    digitalWrite(pinoEnable, HIGH);
  }
  // pixels.setPixelColor(0, pixels.Color(255, 0, 255));  // Purple
  // pixels.show();

  // Configuração OTA
  ArduinoOTA.setHostname("TocaDiscos-Gian");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
    delay(500);

  DEBUG_PRINT("Wifi...OK");
  DEBUG_PRINT(WiFi.localIP().toString().c_str());

  telnet.begin(); // Inicia servidor Telnet na porta 23 padrão
  telnet.println("Use como Serial Monitor remoto.");

  DEBUG_PRINT("Telnet...OK");

  MDNS.begin("tocadiscos");

  // Rotas do Servidor
  server.on("/", HTTP_GET, []() {
    DEBUG_PRINT("On / handler...");
    File file = LittleFS.open("/index.html", "r");
    if (!file) {
      DEBUG_PRINT("404 error...");
      server.send(404, "text/plain",
                  "Erro 404: Arquivo index.html não encontrado! Você esqueceu "
                  "de fazer o Upload Filesystem Image?");
      return;
    }
    server.streamFile(file, "text/html");
    file.close();
  });

  server.on("/get_config", HTTP_GET, []() {
    String json = "{";
    json += "\"liftMax\":" + String(posicaoLiftMax, 1) + ",";
    json += "\"liftMin\":" + String(posicaoLiftMin, 1) + ",";
    json += "\"debounceSec\":" + String(DEBOUNCE_DELAY_MS / 1000.0, 2);
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/salvar", []() {
    if (server.hasArg("liftMax") && server.hasArg("liftMin") &&
        server.hasArg("debounce")) {
      float newMax = server.arg("liftMax").toFloat();
      float newMin = server.arg("liftMin").toFloat();
      float newDebounceSec = server.arg("debounce").toFloat();
      if (newMax >= 0 && newMax <= 180 && newMin >= 0 && newMin <= 180 &&
          newMax < newMin && newDebounceSec >= 0.5 && newDebounceSec <= 10.0) {
        prefs.begin("config", false);
        prefs.putFloat("liftMax", newMax);
        prefs.putFloat("liftMin", newMin);
        prefs.putULong("debounceMs", (unsigned long)(newDebounceSec * 1000));
        prefs.end();
        posicaoLiftMax = newMax;
        posicaoLiftMin = newMin;
        DEBOUNCE_DELAY_MS = (unsigned long)(newDebounceSec * 1000);
        telnet.printf(
            "Config salva: liftMax=%.1f, liftMin=%.1f, debounce=%lu ms\n",
            posicaoLiftMax, posicaoLiftMin, DEBOUNCE_DELAY_MS);
        server.send(200, "text/plain", "Configurações salvas com sucesso!");
      } else {
        server.send(400, "text/plain",
                    "Valores inválidos. Verifique os limites.");
      }
    } else {
      server.send(400, "text/plain", "Parâmetros ausentes.");
    }
  });

  DEBUG_PRINT("Server...OK");

  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS Mount Failed");
    DEBUG_PRINT("LitleFS...Mount Failed");
  } else {
    server.serveStatic("/", LittleFS, "/");
  }
  DEBUG_PRINT("LitleFS...OK");

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  DEBUG_PRINT("Websocket...OK");

  ArduinoOTA.onStart([]() {
    atualizando = true;
    digitalWrite(pinoEnable, HIGH);
  });
  ArduinoOTA.begin();
  server.begin();

  DEBUG_PRINT("OTA...OK");

  // Carregar configurações salvas da NVS
  prefs.begin("config", false); // namespace "config"

  posicaoLiftMax = prefs.getFloat("liftMax", 80.0f); // default 80.0 se não existir
  posicaoLiftMin = prefs.getFloat("liftMin", 120.0f);       // default 120.0
  DEBOUNCE_DELAY_MS = prefs.getULong("debounceMs", 2000UL); // default 2000 ms

  prefs.end();

  DEBUG_PRINT("Config load...OK");

  telnet.printf(
      "Config carregada: liftMax=%.1f°, liftMin=%.1f°, debounce=%lu ms\n",
      posicaoLiftMax, posicaoLiftMin, DEBOUNCE_DELAY_MS);

  // Atualiza posicaoServo inicial com o valor salvo
  posicaoServo = posicaoLiftMax;
  // Serial.println("Início setup - antes de Wire");

  // Wire.begin(tonearmPin_SDA,tonearmPin_SCL); // SDA, SCL

#if (defined(CONFIG_IDF_TARGET_ESP32C3) || defined(ARDUINO_ESP32C3_DEV))
   tonearm.initializeI2C(&Wire);
#elif (defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV))
  tonearm.begin();
#elif 0
  tonearm.begin();
#endif

  DEBUG_PRINT("Tonearm...OK");

  // Serial.println("tonearm.begin OK");
  // setRPM(targetRPM);
  // accelerateTo(targetRPM,3000);
  // startRampTo(targetRPM,3);
}

void loop() {
  webSocket.loop();

  if (millis() - lastStatusUpdate > 200) {
    lastStatusUpdate = millis();
    broadcastStatus();
  }

  ArduinoOTA.handle();
  server.handleClient();
  telnet.loop(); // Mantém o Telnet vivo
                 /*
                   bool wifiConectadoAgora = (WiFi.status() == WL_CONNECTED);
               
                   if (wifiConectadoAgora && !wifiConectadoAnterior) {
                     // WiFi acabou de conectar → acende o LED (ex: verde)
                     //pixels.setPixelColor(0, pixels.Color(0, 255, 0));  // Verde (R,G,B)
                     //pixels.show();
                     //Serial.println("WiFi conectado! LED verde aceso.");
                   }
                   else if (!wifiConectadoAgora && wifiConectadoAnterior) {
                     // WiFi desconectou → apaga ou muda cor (ex: vermelho)
                     //pixels.setPixelColor(0, pixels.Color(255, 0, 0));  // Vermelho
                     //pixels.show();
                     //Serial.println("WiFi desconectado! LED vermelho.");
                   }
                   else if (!wifiConectadoAgora) {
                     // Sem conexão → talvez pisque devagar ou apague
                     // pixels.clear();
                     // pixels.show();
                   }
               
                   wifiConectadoAnterior = wifiConectadoAgora;
                 */
  //float tonearmAngle = 0;
  // acionar o motor conforme o angulo do braço
  #if (defined(CONFIG_IDF_TARGET_ESP32C3) || defined(ARDUINO_ESP32C3_DEV))
    float tonearmAngle = tonearm.angleRead();
  #elif (defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV))
    float tonearmAngle = tonearm.getAngleDegrees();
  #elif 0
    float tonearmAngle = tonearm.angleRead();
  #endif

  //DEBUG_PRINTF("angulo tonearm: %.1f\n", tonearmAngle); // Mantenha para debug

  // Sempre resetar finalDisco quando o braço for levantado (>160°)
  // Isso permite religar depois de um "fim de disco" se o usuário levantar e abaixar novamente
  if (tonearmAngle > 160.0) {
    if (finalDisco) {
      DEBUG_PRINT("Braço levantado → resetando finalDisco para permitir novo play");
      finalDisco = false;
    }
  }
  // Lógica de DESLIGAR (imediata, sem debounce - segurança primeiro)
  if (motorLigado && (tonearmAngle > 160.0 || tonearmAngle < 125.0)) {
    DEBUG_PRINT("angulo DESLIGANDO");
    toggleMotor(false);

    if (tonearmAngle < 125.0) {
      DEBUG_PRINT("FINAL DISCO");
      finalDisco = true;
    }

    // Reset debounce ao levantar
    lowAngleStartTime = 0;
    debounceLowAngleActive = false;
  }

  // Lógica de LIGAR com debounce de 2s (só se ângulo <=160° e >=125° por tempo
  // contínuo)
  else if (!motorLigado && !finalDisco) {
    if (tonearmAngle <= 160.0 && tonearmAngle >= 125.0) {
      if (!debounceLowAngleActive) {
        // Começa a contar agora
        lowAngleStartTime = millis();
        debounceLowAngleActive = true;
        DEBUG_PRINT("Iniciando debounce: tonearm baixado detectado");
      }

      // Verifica se já passou 2s contínuos
      if (millis() - lowAngleStartTime >= DEBOUNCE_DELAY_MS) {
        DEBUG_PRINT("Debounce OK - LIGANDO motor após 2s");
        toggleMotor(true);

        // Reset debounce após ligar
        lowAngleStartTime = 0;
        debounceLowAngleActive = false;
      }
    } else {
      // Saiu da faixa → cancela debounce
      lowAngleStartTime = 0;
      debounceLowAngleActive = false;
    }
  }

  checkThermalStatus();

  // driver.microsteps(32);
  // uint16_t msread=driver.microsteps();
  // DEBUG_PRINT(" read:ms=");
  // DEBUG_PRINT(msread);
}
