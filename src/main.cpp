#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <TMCStepper.h>
#include <ESPTelnet.h>
//#include <Adafruit_NeoPixel.h>

#if (defined(CONFIG_IDF_TARGET_ESP32C3) || defined(ARDUINO_ESP32C3_DEV)) 
  // Alternativa: macro que ignora o parâmetro core
  #define xTaskCreatePinnedToCore(task, name, stack, param, prio, handle, core) \
    xTaskCreate(task, name, stack, param, prio, handle)
#endif

#include <Wire.h>
#include <Preferences.h>

#if (defined(CONFIG_IDF_TARGET_ESP32C3) || defined(ARDUINO_ESP32C3_DEV))
  //ESP32-ESP32S2-AnalogWrite
  // #include <pwmWrite.h>
  //Pwm oServo = Pwm();
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

#define DEBUG_PRINT(x) \
  do { \
    telnet.println(x); \
  } while (0)
#define DEBUG_PRINTF(...) \
  do { \
    telnet.printf(__VA_ARGS__); \
  } while (0)


// Defina o pino e número de LEDs (geralmente 1 no onboard)
#define LED_PIN    8    // Tente 48 primeiro (mais comum no N8)
// #define LED_PIN 38    // Se não funcionar com 48, teste 38 (algumas revisões v1.1)
#define NUM_LEDS   1

//Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

bool wifiConectadoAnterior = false;

// Velocidade desejada (RPM) - você pode mudar via Serial
volatile float targetRPM = 33.33;  // ← ALTERE AQUI ou via Serial Monitor
float currentRPM = 0.0;
float targetRPMSet = 0.0;
unsigned long lastRampUpdate = 0;
const unsigned long RAMP_INTERVAL_MS = 20;   // atualiza a cada 20 ms
float rampIncrementPerStep = 0;              // calculado quando inicia rampa
bool isRamping = false;


static const int servoPin = 10;

static const int tonearmPin_SDA = 6;
static const int tonearmPin_SCL = 7;

bool motorLigado = false;  // Estado inicial
bool posicaoLift = true;  // Estado inicial
float posicaoLiftMin = 120.0; // baixado
float posicaoLiftMax = 80.0; // levantado
bool finalDisco = false;

// Debounce para ligar motor ao baixar tonearm
unsigned long lowAngleStartTime = 0;      // Timestamp quando ângulo primeiro <=160°
unsigned long DEBOUNCE_DELAY_MS = 1500;  // 2 segundos
bool debounceLowAngleActive = false;      // Flag para rastrear se estamos contando tempo

// STEPPER MOTOR

const float CLOCK_CORRECTION = 1.01099f;  // comece com esse valor e ajuste ±0.001 até bater exato
// Motor NEMA17 padrão (200 passos/volta)
#define FULL_STEPS 200
#define MICROSTEPS 256
// Pinos UART no ESP32 (half-duplex)
#define UART_RX_PIN 5
#define UART_TX_PIN 4

// Corrente RMS (ajuste conforme seu motor - comece baixo!)
#define RMS_CURRENT_MA 700  // Ex: 400-800mA para NEMA17 comum

// Pinos UART para o TMC2209
#define DRIVER_ADDRESS 0b00  // Endereço padrão
#define R_SENSE 0.11f        // Valor padrão para drivers StepStick

// Pinos
//const int pinoDirecao = 0;
//const int pinoPasso = 1;
const int pinoEnable = 2;

HardwareSerial mySerial(1);
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
float posicaoServo = posicaoLiftMax;
bool atualizando = false;

void moveServo(float angle, unsigned long time, bool hold) {
  //if (chipModel == "ESP32-C3") {
  #if (defined(CONFIG_IDF_TARGET_ESP32C3) || defined(ARDUINO_ESP32C3_DEV))

    // Runtime logic for C3
    //oServo.write(posicaoServo, time);        
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
  }
  else
  {
    digitalWrite(pinoEnable, LOW);
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

void handleManifest() {
  String manifest = R"=====({
  "name": "Toca-Discos",
  "short_name": "Toca-Discos",
  "start_url": "/",
  "display": "standalone",
  "background_color": "#121212",
  "theme_color": "#121212",
  "icons": [
    {
      "src": "data:image/svg+xml,%3csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 100 100'%3e%3ccircle cx='50' cy='50' r='45' fill='%23333'/%3e%3ccircle cx='50' cy='50' r='35' fill='%23111'/%3e%3ctext x='50' y='55' text-anchor='middle' fill='%23aaa' font-size='20'%3eGIAN%3c/text%3e%3c/svg%3e",
      "sizes": "192x192",
      "type": "image/svg+xml"
    }
  ]
})=====";
  server.send(200, "application/json", manifest);
}

void handleServiceWorker() {
  String sw = R"=====(
const CACHE_NAME = 'tocadiscos-v1';
const urlsToCache = [
  '/',
  '/manifest.json'
];

self.addEventListener('install', event => {
  event.waitUntil(
    caches.open(CACHE_NAME)
      .then(cache => cache.addAll(urlsToCache))
  );
});

self.addEventListener('activate', event => {
  event.waitUntil(
    caches.keys().then(cacheNames => {
      return Promise.all(
        cacheNames.map(cacheName => {
          if (cacheName !== CACHE_NAME) {
            return caches.delete(cacheName);
          }
        })
      );
    })
  );
});

self.addEventListener('fetch', event => {
  event.respondWith(
    caches.match(event.request)
      .then(response => {
        if (response) {
          return response;
        }
        return fetch(event.request);
      })
  );
});
)=====";
  server.send(200, "application/javascript", sw);
}

// Interface Web (HTML/JavaScript)
void handleRoot() {
  String corBotao = motorLigado ? "#e74c3c" : "#2ecc71";
  String textoBotao = motorLigado ? "DESLIGAR MOTOR" : "LIGAR MOTOR";

  String html = "<html charset='utf-8'><head><meta name='viewport' content='width=device-width, initial-scale=1.0'><link rel='manifest' href='/manifest.json'>";
  html += "<style>body{font-family:sans-serif; text-align:center; background:#121212; color:white;} .slider{width:80%; margin:20px;} .btn{padding:15px 30px; margin:10px; font-size:18px; cursor:pointer; border:none; border-radius:5px;} .btn-toggle{background:" + corBotao + "; color:white; width:80%; font-weight:bold;}</style></head>";
  html += "<body><h1>Toca-Discos</h1>";
  // Botão de Power
  html += "<button class='btn btn-toggle' onclick=\"location.href='/toggle'\">" + textoBotao + "</button><br><br>";

  html += "<div><button class='btn' style='background:#444; color:white;' onclick=\"location.href='/set?rpm=33'\">33 1/3 RPM</button>";
  html += "<button class='btn' style='background:#444; color:white;' onclick=\"location.href='/set?rpm=45'\">45 RPM</button></div>";

    // ===================== VISUALIZADOR SVG =====================
  html += "<br><br><h3>Visualizador em Tempo Real</h3>";
  html += "<div style='background:#0a0a0a; padding:15px; border-radius:15px; display:inline-block;'>";
  html += R"=====(
<svg id="turntable" width="720" height="520" viewBox="0 0 520 520" xmlns="http://www.w3.org/2000/svg">

  <!-- Base do toca-discos 
  <circle cx="260" cy="260" r="245" fill="#1c1c1c" stroke="#333" stroke-width="45"/>-->

  <!-- Platter Group (gira) -->
  <g id="platter">
    <circle cx="160" cy="260" r="205" fill="#111" stroke="#444" stroke-width="5"/>
    
    <!-- Ranhuras do vinil -->
    <circle cx="160" cy="260" r="185" fill="none" stroke="#222" stroke-width="7"/>
    <circle cx="160" cy="260" r="165" fill="none" stroke="#222" stroke-width="6"/>
    <circle cx="160" cy="260" r="145" fill="none" stroke="#222" stroke-width="5"/>
    <circle cx="160" cy="260" r="125" fill="none" stroke="#222" stroke-width="4"/>
    <circle cx="160" cy="260" r="105" fill="none" stroke="#222" stroke-width="3"/>
    
    <!-- Etiqueta central -->
    <circle cx="160" cy="260" r="62" fill="#1a1a1a"/>
    <circle cx="160" cy="260" r="48" fill="#333"/>
    <text x="160" y="270" text-anchor="middle" fill="#aaa" font-size="22" font-family="Arial">GIAN</text>
  </g>

  <!-- Eixo central -->
  <circle cx="160" cy="260" r="14" fill="#555"/>

  <!-- Pivô do tonearm -->
  <circle cx="445" cy="155" r="38" fill="#222" stroke="#555" stroke-width="12"/>

  <!-- Tonearm completo -->
  <g id="tonearm" transform="rotate(-42 445 155)">
    <!-- Contrapeso -->
    <rect x="425" y="95" width="48" height="28" rx="8" fill="#333"/>
    
    <!-- Braço principal -->
    <line x1="445" y1="155" x2="180" y2="270" stroke="#1f1f1f" stroke-width="17" stroke-linecap="round"></line>
    
    <!-- Headshell 
    <polygon points="235,235 205,205 255,195" fill="#555" stroke="#222" stroke-width="6"/> -->
    <rect x="155" y="250" width="58" height="28" rx="8" fill="#333"></rect>
    
    <!-- Agulha 
    <line x1="225" y1="220" x2="208" y2="245" stroke="#ffdd44" stroke-width="4" stroke-linecap="round"/>-->
  </g>

</svg>
)=====";
  html += "</div>";

//  html += "<h3>Ajuste Fino (" + String(rpmSelecionado) + " RPM)</h3>";
//  html += "<input type='range' min='0.70' max='1.30' step='0.002' value='" + String(rpmSelecionado > 40 ? ajusteFino45 : ajusteFino33) + "' class='slider' onchange=\"fetch('/ajuste?val='+this.value)\">";
  html += "<h3>Controle do Servo ("+String(posicaoLiftMax)+"-"+String(posicaoLiftMin)+"°)</h3>";
  html += "<input type='range' min='"+String(posicaoLiftMax)+"' max='"+String(posicaoLiftMin)+"' value='" + String(posicaoServo) + "' class='slider' onchange=\"fetch('/servo?pos='+this.value)\">";
  html += "<p>Posição atual: <span id='pos'>" + String(posicaoServo) + "</span>°</p>";
  html += "<br><br><button class='btn' style='background:#3498db; color:white; width:80%;' ";
  html += "onclick=\"location.href='/config'\">CONFIGURAÇÕES AVANÇADAS</button>";

   html += R"=====(
<style>
  #platter {
    transform-origin: 160px 260px;
    animation: spin 1.8s linear infinite;
    animation-play-state: paused;
  }
  @keyframes spin {
    from { transform: rotate(0deg); }
    to   { transform: rotate(360deg); }
  }
</style>

<script>
function map(val, inMin, inMax, outMin, outMax) {
  return (val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

function updateVisualizer() {
  fetch('/status')
    .then(r => r.json())
    .then(data => {
      // Girar o disco conforme RPM
      const speed = data.rpm > 40 ? 1.333 : 1.8;           // 45 ou 33.3 RPM
      document.getElementById('platter').style.animationDuration = speed + 's';
      document.getElementById('platter').style.animationPlayState = data.motorLigado ? 'running' : 'paused';

      // Mover o tonearm (ajuste os números conforme seus valores reais do MT6701)
      const visualAngle = map(data.tonearmAngle, 179, 125, -70, -20);   // ← AJUSTE AQUI se necessário
      document.getElementById('tonearm').setAttribute('transform', 
        `rotate(${visualAngle} 445 155)`);
    });
}

// Atualiza a cada 180ms (suave e leve)
setInterval(updateVisualizer, 180);

if ('serviceWorker' in navigator) {
  navigator.serviceWorker.register('/sw.js')
    .then(reg => console.log('SW registered'))
    .catch(err => console.log('SW registration failed'));
}
</script>
)=====";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

void toggleMotor( bool ligar = false)
{  
  int8_t result = driver.test_connection();
  
  if (result == 0) {
    DEBUG_PRINT("SUCESSO: UART comunicando!");
    Serial.print("SUCESSO: UART comunicando!");    
  } else {
    DEBUG_PRINTF("ERRO: Falha na comunicação (Código: %d)\n", result);
    Serial.printf("ERRO: Falha na comunicação (Código: %d)\n", result);
    DEBUG_PRINT("Verifique: 1. Alimentação VMOT (12V) ligada? 2. Resistor de 1k? 3. Pinos TX/RX invertidos?");
    digitalWrite(pinoEnable, HIGH);
  }
  if (ligar)
  {
    //startRampTo(targetRPM,3);
    posicaoServo = posicaoLiftMin;
    //ledcDetachPin(servoPin);
    moveServo(posicaoServo,1600,true);
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
    
  }
  else
  {
    posicaoServo = posicaoLiftMax;
    moveServo(posicaoServo,400,false);
    posicaoLift = true;

    // desliga motor
    setRPM(0);    
    //delay(4100);  // ou melhor: use um timer não bloqueante
    //ledcDetachPin(servoPin);
  }
  motorLigado = !motorLigado;
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

  Serial.begin(115200);  // use Serial normal do ESP32 para debug

  #if (defined(CONFIG_IDF_TARGET_ESP32C3) || defined(ARDUINO_ESP32C3_DEV))
      // Runtime logic for C3
      oServo.attach(servoPin);
  #elif (defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV)) && 0
      // Runtime logic for S3
      //configure our main settings in the ESP32 LEDC registry
      Esp32LedcRegistry::instance()->begin(LEDC_CONFIG_ESP32_S3);		//change this for the relevant 
      BestAvailableFactory oTimerChannelFactory;						//used to select the best available timer & channel based on the hardware setup
      ServoFactoryDecorator oFactoryDecorator(oTimerChannelFactory);	//let this ServoFactoryDecorator define the servo frequency to use and such
      //the above two are needed (variable scope related, in 'begin' only)
      
      //if (!oServo.begin(oFactoryDecorator, servoPin)) {				//3rd parameter is the default angle to start from: 90 degrees in this case
        //Serial.println("  failed to init the servo..");
        //pixels.setPixelColor(0, pixels.Color(255, 255, 0));
        //pixels.show();
        //return;
      //}
  #elif 0
    // Handle other models
    oServo.attach(servoPin);
  #endif
  
  //pinMode(pinoDirecao, OUTPUT);
  //pinMode(pinoPasso, OUTPUT);
  pinMode(pinoEnable, OUTPUT);

  //digitalWrite(pinoDirecao, LOW);
  digitalWrite(pinoEnable, HIGH); // deixar desligado
  //digitalWrite(pinoPasso, LOW);


  //pixels.begin();           // Inicializa o NeoPixel
  //pixels.setBrightness(50); // 0-255, comece baixo para não ofuscar (50 é bom)
  //pixels.clear();           // Apaga o LED no início
  //pixels.setPixelColor(0, pixels.Color(0, 0, 255));  // Blue
  //pixels.show();

  // Inicia comunicação UART com o Driver
  mySerial.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);


  driver.begin();
  delay(200);
  driver.toff(2);                // Habilita o driver
  driver.rms_current(RMS_CURRENT_MA);
  //driver.mstep_reg_select(1);  // necessary for TMC2208 to set microstep register with UART
  driver.microsteps(MICROSTEPS); 
  driver.I_scale_analog(false);  // Usa corrente via UART (não potenciômetro)

  driver.en_spreadCycle(false);  // DESLIGA SpreadCycle (obrigatório para Stealth)
  driver.pwm_autoscale(false);    // Ativa StealthChop2
  driver.pwm_freq(2);     // Auto-tuning do PWM (ainda mais silencioso)
  driver.pwm_ofs(150);      // Valor de amplitude inicial (ajusta a força do campo magnético parado)
  driver.pwm_grad(4);      // Gradiente de aceleração do PWM
  driver.pwm_autograd(true);     // Auto-tuning do PWM (ainda mais silencioso)

  
  driver.TPWMTHRS(0);            // Fica 100% em StealthChop (sem troca de modo)


   // Ajustes finos para ruído em repouso
  //driver.iholddelay(10);          // Atraso para reduzir corrente em repouso
  //driver.TPOWERDOWN(128);         // Tempo para entrar em modo de economia
  //driver.pwm_temp_stepdown(true); // Proteção térmica que reduz ruído
  
  int8_t result = driver.test_connection();
  
  if (result == 0) {
    DEBUG_PRINT("SUCESSO: UART comunicando!");
    Serial.print("SUCESSO: UART comunicando!");    
  } else {
    DEBUG_PRINTF("ERRO: Falha na comunicação (Código: %d)\n", result);
    Serial.printf("ERRO: Falha na comunicação (Código: %d)\n", result);
    DEBUG_PRINT("Verifique: 1. Alimentação VMOT (12V) ligada? 2. Resistor de 1k? 3. Pinos TX/RX invertidos?");
    digitalWrite(pinoEnable, HIGH);
  }
  //pixels.setPixelColor(0, pixels.Color(255, 0, 255));  // Purple
  //pixels.show();

  // Configuração OTA
  ArduinoOTA.setHostname("TocaDiscos-Gian");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);


  telnet.begin();  // Inicia servidor Telnet na porta 23 padrão
  telnet.println("Use como Serial Monitor remoto.");

  MDNS.begin("tocadiscos");

  // Rotas do Servidor
  server.on("/manifest.json", handleManifest);
  server.on("/sw.js", handleServiceWorker);
  server.on("/", handleRoot);
  server.on("/set", []() {
    float r = server.arg("rpm").toFloat();
    if (r < 40) rpmSelecionado = 33.333;
    else rpmSelecionado = 45.0;
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

    //digitalWrite(pinoEnable, motorLigado ? LOW : HIGH);
    toggleMotor(!motorLigado);

    server.sendHeader("Location", "/");
    server.send(303);
  });
  server.on("/servo", []() {
    if (server.hasArg("pos")) {
      int novaPos = server.arg("pos").toFloat();
      if (novaPos >= posicaoLiftMax && novaPos <= posicaoLiftMin) {
        posicaoServo = novaPos;
        moveServo(posicaoServo,200,false);
        
        // Opcional: atualiza o HTML dinamicamente, mas fetch simples já basta
      }
    }
    server.send(200, "text/plain", String(posicaoServo));
  });

  server.on("/config", HTTP_GET, []() {
    // Mostra página de configuração
    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<style>body{font-family:sans-serif; text-align:center; background:#121212; color:white;}";
    html += "input, button{margin:10px; padding:10px; font-size:16px; width:80%; max-width:400px;}</style></head>";
    html += "<body><h1>Configurações do Toca-Discos</h1>";

    html += "<h3>Posição Servo Levantado (liftMax)</h3>";
    html += "<input type='number' id='liftMax' step='0.1' value='" + String(posicaoLiftMax, 1) + "'>";

    html += "<h3>Posição Servo Baixado (liftMin)</h3>";
    html += "<input type='number' id='liftMin' step='0.1' value='" + String(posicaoLiftMin, 1) + "'>";

    html += "<h3>Tempo para ligar motor (debounce em segundos)</h3>";
    html += "<input type='number' id='debounceSec' step='0.1' min='0.5' max='10' value='" + String(DEBOUNCE_DELAY_MS / 1000.0, 2) + "'>";

    html += "<br><button onclick='salvar()'>SALVAR CONFIGURAÇÕES</button>";
    html += "<p id='status'></p>";

    html += "<script>";
    html += "function salvar() {";
    html += "  let liftMax = document.getElementById('liftMax').value;";
    html += "  let liftMin = document.getElementById('liftMin').value;";
    html += "  let debounceSec = document.getElementById('debounceSec').value;";
    html += "  fetch('/salvar?liftMax=' + liftMax + '&liftMin=' + liftMin + '&debounce=' + debounceSec)";
    html += "    .then(r => r.text()).then(txt => {";
    html += "      document.getElementById('status').innerText = txt;";
    html += "    });";
    html += "}";
    html += "</script></body></html>";

    server.send(200, "text/html", html);
  });

  server.on("/salvar", []() {
    if (server.hasArg("liftMax") && server.hasArg("liftMin") && server.hasArg("debounce")) {
      float newMax = server.arg("liftMax").toFloat();
      float newMin = server.arg("liftMin").toFloat();
      float newDebounceSec = server.arg("debounce").toFloat();

      // Validação básica
      if (newMax >= 0 && newMax <= 180 && newMin >= 0 && newMin <= 180 && newMax < newMin &&
          newDebounceSec >= 0.5 && newDebounceSec <= 10.0) {

        prefs.begin("config", false);
        prefs.putFloat("liftMax", newMax);
        prefs.putFloat("liftMin", newMin);
        prefs.putULong("debounceMs", (unsigned long)(newDebounceSec * 1000));
        prefs.end();

        // Atualiza variáveis em tempo real
        posicaoLiftMax = newMax;
        posicaoLiftMin = newMin;
        DEBOUNCE_DELAY_MS = (unsigned long)(newDebounceSec * 1000);

        telnet.printf("Config salva: liftMax=%.1f, liftMin=%.1f, debounce=%lu ms\n",
                      posicaoLiftMax, posicaoLiftMin, DEBOUNCE_DELAY_MS);

        server.send(200, "text/plain", "Configurações salvas com sucesso!");
          
        //delay(3000);
        //ESP.restart();
      } else {
        server.send(400, "text/plain", "Valores inválidos. Verifique os limites.");
      }
    } else {
      server.send(400, "text/plain", "Parâmetros ausentes.");
    }
  });

    // ===================== NOVA ROTA STATUS =====================
  server.on("/status", []() {
    String json = "{";
    json += "\"motorLigado\":" + String(motorLigado ? "true" : "false") + ",";
    // acionar o motor conforme o angulo do braço
    #if (defined(CONFIG_IDF_TARGET_ESP32C3) || defined(ARDUINO_ESP32C3_DEV))    
      json += "\"tonearmAngle\":" + String(tonearm.angleRead(), 1) + ",";
    #elif (defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV)) && 0
      json += "\"tonearmAngle\":" + String(tonearm.getAngleDegrees(), 1) + ",";
    #elif 0
      json += "\"tonearmAngle\":" + String(tonearm.angleRead(), 1) + ",";
    #endif
    
    json += "\"rpm\":" + String(rpmSelecionado, 1);
    json += "}";
    server.send(200, "application/json", json);
  });

  ArduinoOTA.onStart([]() {
    atualizando = true;
    digitalWrite(pinoEnable, HIGH);
  });
  ArduinoOTA.begin();
  server.begin();

  // Carregar configurações salvas da NVS
  prefs.begin("config", false);  // namespace "config"

  posicaoLiftMax = prefs.getFloat("liftMax", 80.0f);     // default 80.0 se não existir
  posicaoLiftMin = prefs.getFloat("liftMin", 120.0f);    // default 120.0
  DEBOUNCE_DELAY_MS = prefs.getULong("debounceMs", 2000UL);  // default 2000 ms

  prefs.end();

  telnet.printf("Config carregada: liftMax=%.1f°, liftMin=%.1f°, debounce=%lu ms\n",
                posicaoLiftMax, posicaoLiftMin, DEBOUNCE_DELAY_MS);

  // Atualiza posicaoServo inicial com o valor salvo
  posicaoServo = posicaoLiftMax;
  //Serial.println("Início setup - antes de Wire");

  Wire.begin(tonearmPin_SDA,tonearmPin_SCL); // SDA, SCL
  //Serial.println("Wire.begin OK");
  ////Wire.setClock(400000);
  
  #if (defined(CONFIG_IDF_TARGET_ESP32C3) || defined(ARDUINO_ESP32C3_DEV))
    tonearm.initializeI2C(&Wire);
  #elif (defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV))
    tonearm.begin();
  #elif 0
    tonearm.begin();
  #endif
  
  //Serial.println("tonearm.begin OK");
  //setRPM(targetRPM);
  //accelerateTo(targetRPM,3000);
  //startRampTo(targetRPM,3);

}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  telnet.loop();  // Mantém o Telnet vivo
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

  // acionar o motor conforme o angulo do braço
  #if (defined(CONFIG_IDF_TARGET_ESP32C3) || defined(ARDUINO_ESP32C3_DEV))    
    float tonearmAngle = tonearm.angleRead();
  #elif (defined(CONFIG_IDF_TARGET_ESP32S3) || defined(ARDUINO_ESP32S3_DEV))
    float tonearmAngle = tonearm.getAngleDegrees();
  #elif 0
    float tonearmAngle = tonearm.angleRead();
  #endif
  
  //DEBUG_PRINTF("angulo tonearm: %.1f\n", tonearmAngle);  // Mantenha para debug

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

  // Lógica de LIGAR com debounce de 2s (só se ângulo <=160° e >=125° por tempo contínuo)
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

  //driver.microsteps(32);   
  //uint16_t msread=driver.microsteps();
  //DEBUG_PRINT(" read:ms=");  
  //DEBUG_PRINT(msread); 
}


