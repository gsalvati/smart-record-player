#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <TMCStepper.h> // Instale esta biblioteca no Gerenciador de Bibliotecas
#include <ESPTelnet.h>

// Pinos
const int pinoDirecao = 1;
const int pinoPasso = 2;
const int pinoEnable = 3;

ESPTelnet telnet;

#define DEBUG_PRINT(x)   do { Serial.println(x); telnet.println(x); } while(0)
#define DEBUG_PRINTF(...) do { Serial.printf(__VA_ARGS__); telnet.printf(__VA_ARGS__); } while(0)

// Pinos UART para o TMC2209
#define RXD2 16 // PDN/UART do driver
#define TXD2 17 // Não usado diretamente, mas necessário para Serial2
#define DRIVER_ADDRESS 0b00 // Endereço padrão
#define R_SENSE 0.11f // Valor padrão para drivers StepStick

TMC2209Stepper driver(&Serial2, R_SENSE, DRIVER_ADDRESS);

bool motorLigado = true; // Estado inicial

// Configuração Wifi
const char* ssid = "SALVATI";
const char* password = "2007112402";

WebServer server(80);

// CONFIGURAÇÃO DO MOTOR (Ajuste aqui)
// Se com 12800 está lento (18 RPM), tente mudar para 6400 ou 3200
double npassosVolta = 6400.0;

// Variáveis de Controle
float rpmSelecionado = 33.333;
float ajusteFino33 = 1.0; 
float ajusteFino45 = 1.0;
float intervaloAtual;
float intervaloRampa = 1000.0; // Começa bem mais devagar que o alvo

unsigned long tempoAnterior = 0;
double acumuladorMicros = 0;
bool atualizando = false;

// Função para calcular o intervalo real
void calcularIntervalo() {
  float rpmAlvo = (rpmSelecionado > 40) ? 45.0 : 33.3333;
  float ajuste = (rpmSelecionado > 40) ? ajusteFino45 : ajusteFino33;
  intervaloAtual = (60.0 / (rpmAlvo * npassosVolta)) * 1000000.0;
  intervaloAtual = intervaloAtual * ajuste;
}

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
  html += "<input type='range' min='0.90' max='1.10' step='0.0005' value='" + String(rpmSelecionado > 40 ? ajusteFino45 : ajusteFino33) + "' class='slider' onchange=\"fetch('/ajuste?val='+this.value)\">";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void setup() {
  pinMode(pinoDirecao, OUTPUT);
  pinMode(pinoPasso, OUTPUT);
  pinMode(pinoEnable, OUTPUT);

  digitalWrite(pinoEnable, LOW);
  digitalWrite(pinoDirecao, LOW);

  // Inicia comunicação UART com o Driver
  //Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  

  // driver.begin();
  // driver.toff(4);                 // Habilita o driver
  // driver.blank_time(24);
  // driver.rms_current(800);        // Ajusta a corrente (800mA é um bom ponto de partida)
  // driver.microsteps(16);          // CONFIGURAÇÃO 1/64 VIA SOFTWARE
  // driver.pwm_autoscale(true);     // Necessário para StealthChop
  // driver.en_spreadCycle(false);   // Força o modo silencioso (StealthChop)

  Serial.begin(115200);  // use Serial normal do ESP32 para debug
  // Configuração OTA
  ArduinoOTA.setHostname("tocadiscos-gian");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  telnet.begin();  // Inicia servidor Telnet na porta 23 padrão

  // Opcional: saudação quando alguém conecta
  //telnet.onConnect("Conectado ao Toca-Discos Gian! IP: " + WiFi.localIP().toString());
  telnet.println("Use como Serial Monitor remoto.");
  
  MDNS.begin("tocadiscos");
  
  // Rotas do Servidor
  server.on("/", handleRoot);
  server.on("/set", []() {
    float r = server.arg("rpm").toFloat();
    if(r < 40) rpmSelecionado = 33.333; else rpmSelecionado = 45.0;
    calcularIntervalo();
    server.sendHeader("Location", "/"); server.send(303);
  });
  server.on("/ajuste", []() {
    float val = server.arg("val").toFloat();
    if(rpmSelecionado < 40) ajusteFino33 = val; else ajusteFino45 = val;
    calcularIntervalo();
    server.send(200, "text/plain", "OK");
  });
  // Rota para Ligar/Desligar o motor
    server.on("/toggle", []() {

    //if (driver.test_connection()) {
    //  DEBUG_PRINT("UART TMC OK!");
    //} else {
    //  DEBUG_PRINT("ERRO: TMC não responde via UART!");
    //}
    
    //DEBUG_PRINT("TMC2209 inicializado. Corrente: " + String(driver.rms_current()));
    //DEBUG_PRINTF("Intervalo calculado: %.2f us\n", intervaloAtual);
    //DEBUG_PRINT("Microstep: " + String(driver.microsteps()));

    motorLigado = !motorLigado;
    // No TMC2209: LOW = Ativado, HIGH = Desativado (bobinas soltas)
    digitalWrite(pinoEnable, motorLigado ? LOW : HIGH); 
    server.sendHeader("Location", "/"); 
    server.send(303);
  });
  ArduinoOTA.onStart([]() { atualizando = true; digitalWrite(pinoEnable, HIGH); });
  ArduinoOTA.begin();
  server.begin();
  calcularIntervalo();
}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  telnet.loop();  // Mantém o Telnet vivo
  
  if (atualizando || !motorLigado) {
    tempoAnterior = micros(); // Mantém o tempo atualizado para não dar pulo ao ligar
    return;
  }

  unsigned long tempoAtual = micros();
  unsigned long deltaTempo = tempoAtual - tempoAnterior;
  tempoAnterior = tempoAtual;
  acumuladorMicros += (double)deltaTempo;

  //if (acumuladorMicros >= intervaloAtual) {
    digitalWrite(pinoPasso, HIGH);
    delayMicroseconds(intervaloAtual);
    digitalWrite(pinoPasso, LOW);
    acumuladorMicros -= intervaloAtual;
  //}



}