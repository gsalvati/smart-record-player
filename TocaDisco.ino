#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

// Pinos
const int pinoDirecao = 1;
const int pinoPasso = 2;
const int pinoEnable = 3;

bool motorLigado = true; // Estado inicial

// Configuração Wifi
const char* ssid = "SALVATI";
const char* password = "2007112402";

WebServer server(80);

// TMC2209
// Variáveis de Controle
float rpmSelecionado = 33.333;
float ajusteFino33 = 1.0;
float ajusteFino45 = 1.0;
float intervaloAtual;
//double npassosVolta = 12800;
double npassosVolta = 13075;


unsigned long tempoAnterior = 0;
double acumuladorMicros = 0;
bool atualizando = false;

// Função para calcular o intervalo real
void calcularIntervalo() {
  float base = (rpmSelecionado > 40) ? (60.0 / (45.0 * npassosVolta) * 1000000.0) : (60.0 / (33.333 * npassosVolta) * 1000000.0);
  float ajuste = (rpmSelecionado > 40) ? ajusteFino45 : ajusteFino33;
  intervaloAtual = base * ajuste;
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

  // Configuração OTA
  ArduinoOTA.setHostname("TocaDiscos-Gian");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);

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
  //ArduinoOTA.handle();
  //server.handleClient();
  if (atualizando) return;

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