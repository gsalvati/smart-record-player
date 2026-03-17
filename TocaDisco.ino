#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <TMCStepper.h>
#include <ESP32Servo.h>
#include <ESPTelnet.h>

ESPTelnet telnet;

Servo lift;

#define DEBUG_PRINT(x) \
  do { \
    Serial.println(x); \
    telnet.println(x); \
  } while (0)
#define DEBUG_PRINTF(...) \
  do { \
    Serial.printf(__VA_ARGS__); \
    telnet.printf(__VA_ARGS__); \
  } while (0)

const float CLOCK_CORRECTION = 1.012f;  // comece com esse valor e ajuste ±0.001 até bater exato
// Motor NEMA17 padrão (200 passos/volta)
#define FULL_STEPS 200
#define MICROSTEPS 256
// Pinos UART no ESP32 (half-duplex)
#define UART_RX_PIN 16
#define UART_TX_PIN 17

// Motor NEMA17 padrão (200 passos/volta)
#define FULL_STEPS 200
#define MICROSTEPS 256

// Corrente RMS (ajuste conforme seu motor - comece baixo!)
#define RMS_CURRENT_MA 400  // Ex: 400-800mA para NEMA17 comum

// Velocidade desejada (RPM) - você pode mudar via Serial
volatile float targetRPM = 33.33;  // ← ALTERE AQUI ou via Serial Monitor
float currentRPM = 0.0;
float targetRPMSet = 0.0;
unsigned long lastRampUpdate = 0;
const unsigned long RAMP_INTERVAL_MS = 20;   // atualiza a cada 20 ms
float rampIncrementPerStep = 0;              // calculado quando inicia rampa
bool isRamping = false;

// Pinos
const int pinoDirecao = 1;
const int pinoPasso = 2;
const int pinoEnable = 3;

static const int servoPin = 5;

bool motorLigado = true;  // Estado inicial

// Pinos UART para o TMC2209
#define RXD2 16              // PDN/UART do driver
#define TXD2 17              // Não usado diretamente, mas necessário para Serial2
#define DRIVER_ADDRESS 0b00  // Endereço padrão
#define R_SENSE 0.11f        // Valor padrão para drivers StepStick


HardwareSerial mySerial(2);
TMC2209Stepper driver(&mySerial, R_SENSE, DRIVER_ADDRESS);

// Configuração Wifi
const char* ssid = "SALVATI";
const char* password = "2007112402";

WebServer server(80);

// TMC2209
// Variáveis de Controle
float rpmSelecionado = 33.333;
float ajusteFino33 = 1.0;
float ajusteFino45 = 1.0;

bool atualizando = false;

// Interface Web (HTML/JavaScript)
void handleRoot() {
  String corBotao = motorLigado ? "#e74c3c" : "#2ecc71";
  String textoBotao = motorLigado ? "DESLIGAR MOTOR" : "LIGAR MOTOR";

  String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<style>body{font-family:sans-serif; text-align:center; background:#121212; color:white;} .slider{width:80%; margin:20px;} .btn{padding:15px 30px; margin:10px; font-size:18px; cursor:pointer; border:none; border-radius:5px;} .btn-toggle{background:" + corBotao + "; color:white; width:80%; font-weight:bold;}</style></head>";
  html += "<body><h1>Toca-Discos Gian</h1>";
  // Botão de Power
  html += "<button class='btn btn-toggle' onclick=\"location.href='/toggle'\">" + textoBotao + "</button><br><br>";

  html += "<div><button class='btn' style='background:#444; color:white;' onclick=\"location.href='/set?rpm=33'\">33 1/3 RPM</button>";
  html += "<button class='btn' style='background:#444; color:white;' onclick=\"location.href='/set?rpm=45'\">45 RPM</button></div>";
  html += "<h3>Ajuste Fino (" + String(rpmSelecionado) + " RPM)</h3>";
  html += "<input type='range' min='0.70' max='1.30' step='0.002' value='" + String(rpmSelecionado > 40 ? ajusteFino45 : ajusteFino33) + "' class='slider' onchange=\"fetch('/ajuste?val='+this.value)\">";
  html += "</body></html>";
  server.send(200, "text/html", html);
}


void startRampTo(float newTargetRPM, float accelTimeSeconds = 2.0) {
  targetRPMSet = newTargetRPM;
  
  if (accelTimeSeconds <= 0.05) accelTimeSeconds = 0.05;
  
  float deltaRPM = newTargetRPM - currentRPM;
  float steps = (accelTimeSeconds * 1000.0) / RAMP_INTERVAL_MS;
  
  rampIncrementPerStep = deltaRPM / steps;
  
  isRamping = true;
  lastRampUpdate = millis();
}

void setup() {

  pinMode(pinoDirecao, OUTPUT);
  pinMode(pinoPasso, OUTPUT);
  pinMode(pinoEnable, OUTPUT);

  digitalWrite(pinoDirecao, LOW);

  //lift.attach(servoPin);

  Serial.begin(115200);  // use Serial normal do ESP32 para debug
  // Inicia comunicação UART com o Driver
  mySerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  driver.begin();
  delay(100);
  driver.toff(4);                // Habilita o driver
  driver.en_spreadCycle(false);  // DESLIGA SpreadCycle (obrigatório para Stealth)
  driver.pwm_autoscale(true);    // Ativa StealthChop2
  driver.pwm_autograd(true);     // Auto-tuning do PWM (ainda mais silencioso)
  driver.TPWMTHRS(0);            // Fica 100% em StealthChop (sem troca de modo)
  driver.rms_current(RMS_CURRENT_MA);
  driver.I_scale_analog(false);  // Usa corrente via UART (não potenciômetro)

  
  // Configuração OTA
  ArduinoOTA.setHostname("TocaDiscos-Gian");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);


  telnet.begin();  // Inicia servidor Telnet na porta 23 padrão
  telnet.println("Use como Serial Monitor remoto.");

  MDNS.begin("tocadiscos");

  // Rotas do Servidor
  server.on("/", handleRoot);
  server.on("/set", []() {
    float r = server.arg("rpm").toFloat();
    if (r < 40) rpmSelecionado = 33.333;
    else rpmSelecionado = 45.0;
    //calcularIntervalo();
    targetRPM = rpmSelecionado;
    setRPM(targetRPM);
    server.sendHeader("Location", "/");
    server.send(303);
  });
  server.on("/ajuste", []() {
    float val = server.arg("val").toFloat();
    if (rpmSelecionado < 40) ajusteFino33 = val;
    else ajusteFino45 = val;
    targetRPM = rpmSelecionado * val;
    setRPM(targetRPM);
    server.send(200, "text/plain", "OK");
  });
  // Rota para Ligar/Desligar o motor
  server.on("/toggle", []() {
    DEBUG_PRINT("Microstep: " + String(driver.microsteps()));

    if (driver.test_connection()) {
      DEBUG_PRINT("UART TMC OK!");
    } else {
      DEBUG_PRINT("ERRO: TMC não responde via UART!");
    }

    if (driver.isEnabled()) {
      DEBUG_PRINT("Driver habilitado");
    } else {
      DEBUG_PRINT("Driver desabilitado");
    }

    motorLigado = !motorLigado;
    // if (!motorLigado)
    // {
    //   startRampTo(targetRPM,3);
    // }
    // No TMC2209: LOW = Ativado, HIGH = Desativado (bobinas soltas)
    digitalWrite(pinoEnable, motorLigado ? LOW : HIGH);
    
    server.sendHeader("Location", "/");
    server.send(303);
  });
  ArduinoOTA.onStart([]() {
    atualizando = true;
    digitalWrite(pinoEnable, HIGH);
  });
  ArduinoOTA.begin();
  server.begin();
  //calcularIntervalo();

  digitalWrite(pinoEnable, LOW);
  setRPM(targetRPM);
  //accelerateTo(targetRPM,3000);
  //startRampTo(targetRPM,3);
  
  
}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  telnet.loop();  // Mantém o Telnet vivo


  // unsigned long now = millis();

  // if (isRamping && (now - lastRampUpdate >= RAMP_INTERVAL_MS)) {
  //   currentRPM += rampIncrementPerStep;
    
  //   // Chegou no alvo (ou passou um pouquinho)?
  //   if ((rampIncrementPerStep > 0 && currentRPM >= targetRPMSet) ||
  //       (rampIncrementPerStep < 0 && currentRPM <= targetRPMSet)) {
  //     currentRPM = targetRPMSet;
  //     isRamping = false;
  //   }
    
  //   setRPM(currentRPM);
  //   lastRampUpdate = now;
  // }
  // // for (int posDegrees = 0; posDegrees <= 15; posDegrees++) {
  // //   lift.write(posDegrees);
  //   DEBUG_PRINTF("graus: %.1ff\n",posDegrees);
  //   delay(20);
  // }
  // for (int posDegrees = 15; posDegrees >= 0; posDegrees--) {
  //   lift.write(posDegrees);
  //   DEBUG_PRINTF("graus: %.1ff\n",posDegrees);
  //   delay(20);
  // }
}

// ===================== FUNÇÃO DE RPM =====================
// Calcula VACTUAL corretamente (fCLK = 12 MHz interno)
void setRPM(float rpm) {
  if (rpm == 0) {
    driver.VACTUAL(0);
    return;
  }
  //rpm = rpm * 0.99;

  // Velocidade em microsteps/segundo
  float usteps_per_sec = (rpm / 60.0f) * FULL_STEPS * MICROSTEPS;

  // Fórmula oficial do datasheet TMC2209
  // VACTUAL = usteps/s * (2^24 / 12.000.000)
  //int32_t vactual = (int32_t)round(usteps_per_sec * (16777216.0f / 12000000.0f));
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

  float startRPM = currentRPM;  // começa de onde está
  if (startRPM < 0) startRPM = 0;

  unsigned long startTime = millis();
  unsigned long elapsed;

  do {
    elapsed = millis() - startTime;
    float progress = (float)elapsed / accelTimeMs;
    if (progress > 1.0) progress = 1.0;

    float nowRPM = startRPM + (targetRPM - startRPM) * progress;
    setRPM(nowRPM);           // usa a função setRPM que já existe

    delay(10);                // atualização a cada ~10 ms → suave o suficiente
  } while (elapsed < accelTimeMs);

  currentRPM = targetRPM;     // atualiza a variável global
  setRPM(targetRPM);          // garante o valor final exato
}
