# Smart Record Player Firmware

*Leia isso em outro idioma: [English](README.md)*

Este repositório contém o firmware de um projeto de **Toca-discos Inteligente (Smart Record Player)** baseado no microcontrolador ESP32-C3. Ele utiliza controle de motor de passo de alta precisão, leitura angular avançada e interface web para oferecer uma experiência automatizada e moderna na reprodução de discos de vinil.

## 🌟 Características e Funcionalidades

### 1. Controle Preciso e Silencioso do Prato (TMC2209)
* **Microstepping e StealthChop2**: Utiliza o driver TMC2209 para controlar o motor de passo (ex: NEMA17) de forma ultra-silenciosa e fluida, minimizando ressonâncias e garantindo alta fidelidade de áudio.
* **Ajuste de RPM**: Suporte nativo para 33 ⅓ RPM e 45 RPM, configuráveis remotamente via painel web, com cálculos rigorosos de velocidade enviados via comunicação UART.
* **Rampas de Aceleração**: Conta com aceleração e desaceleração suaves para iniciar e parar a rotação do prato maciamente.

### 2. Leitura Angular do Braço (Tonearm)
* **Sensor Magnético MT6701**: Efetua a leitura absoluta da posição e ângulo do braço do toca-discos utilizando barramento I2C.
* **Detecção Automática**: Consegue identificar a movimentação inicial e detectar o fim do disco de vinil com base na angulação, acionando a parada do motor automaticamente.

### 3. Elevação Automática da Agulha (Tonearm Lift)
* **Servo Motor Integrado**: Atua no mecanismo de elevação (lift) do braço, garantindo que a agulha seja abaixada ou levantada com precisão mecânica e segurança.
* **Operação Integrada**: Ao mover o braço em direção ao prato, após um atraso seguro (debounce programável), o motor é ligado e a agulha desce suavemente. Quando o limite final do disco é detectado, a agulha levanta e a rotação é cessada.

### 4. Interface Web e Monitoramento
* **Web App (Hospedagem Interna)**: Aplicativo PWA (HTML/JS/CSS) totalmente armazenado no próprio microcontrolador graças ao LittleFS.
* **WebSockets em Tempo Real**: Comunicação bidirecional contínua de telemetria, mostrando velocidade atual, ângulo do braço e status do levantador na tela do navegador ou smartphone.
* **Modos de Controle**: Chaveamento entre modo **Automático** ou **Manual** sem necessidade de alterar código.

### 5. Sistema Moderno e Conectividade
* **OTA (Over-The-Air)**: Capacidade de atualização remota de firmware pela rede Wi-Fi, sem necessidade de cabo USB conectado após o aparelho montado.
* **Telnet Debug**: Servidor Telnet embarcado na porta 23 atuando como monitor serial virtual para diagnóstico e acompanhamento prático.
* **Persistência de Dados**: Configurações finas como os limites máximo/mínimo do servo e tempo de debounce são gravadas permanentemente na memória NVS.

## 🛠️ Stack e Hardware
* **Microcontrolador**: Seeed Studio XIAO ESP32C3
* **Driver do Motor**: TMC2209 (Comunicando via UART e modo Legacy)
* **Transdutor de Rotação**: Motor de Passo NEMA17
* **Sensor de Posição**: MT6701 Magnético
* **Atuador de Elevação**: Micro Servo
* **Framework**: PlatformIO com interface Arduino

## 📌 Configuração de Pinos (Pinout)
Conforme mapeamento do projeto:
* **TMC2209 (UART)**: `RX = Pino 5` / `TX = Pino 4` / `Enable = Pino 2`
* **Servo Motor**: `Pino 10`
* **Sensor MT6701 (I2C)**: `SDA = Pino 6` / `SCL = Pino 7`

## ⚙️ Instalação e Deploy (PlatformIO)

1. Clone o repositório e abra a pasta raiz em uma IDE com PlatformIO (como o VS Code).
2. Opcionalmente, preencha suas credenciais de Wi-Fi padrão em `src/main.cpp`.
3. Certifique-se de executar a task **Upload Filesystem Image** (no menu PlatformIO) para enviar a pasta `data/` contendo a interface web para o LittleFS do ESP32.
4. Execute o **Build** e depois o **Upload** da aplicação via cabo.
5. Acesse o painel pelo IP designado ou através de `http://tocadiscos.local` se o mDNS estiver operante em sua rede.
