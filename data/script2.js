let socket;
let reconnectTimer = null;
let heartbeatTimer = null;
let heartbeatMissed = 0;
const HEARTBEAT_INTERVAL = 1500;   // ping a cada 1.5s
const HEARTBEAT_TIMEOUT  = 3000;   // considera morto após 3s sem pong
const RECONNECT_INTERVAL = 1000;   // tenta reconectar a cada 1s
let isOnline = false;
let pendingCommands = [];          // buffer de comandos quando offline

function initWebSocket() {
    if (socket && (socket.readyState === WebSocket.CONNECTING || socket.readyState === WebSocket.OPEN)) {
        return;
    }

    clearTimeout(reconnectTimer);
    reconnectTimer = null;

    socket = new WebSocket(`ws://${window.location.hostname}:81/`);

    socket.onopen = function(e) {
        console.log("WebSocket conectado");
        isOnline = true;
        heartbeatMissed = 0;

        // Envia comandos pendentes que acumularam enquanto offline
        while (pendingCommands.length > 0) {
            const cmd = pendingCommands.shift();
            try { socket.send(JSON.stringify(cmd)); } catch (e) {}
        }

        startHeartbeat();
    };

    socket.onmessage = function(event) {
        heartbeatMissed = 0; // reset heartbeat em qualquer mensagem
        const data = JSON.parse(event.data);
        updateUI(data);
    };

    socket.onclose = function(event) {
        console.log("WebSocket desconectado");
        isOnline = false;
        stopHeartbeat();
        scheduleReconnect();
    };

    socket.onerror = function(error) {
        console.log("WebSocket erro: ", error);
        isOnline = false;
        socket.close();
    };
}

function startHeartbeat() {
    stopHeartbeat();
    heartbeatTimer = setInterval(() => {
        if (!socket || socket.readyState !== WebSocket.OPEN) {
            stopHeartbeat();
            return;
        }
        if (heartbeatMissed >= 2) {
            console.log("Heartbeat falhou, forçando reconexão...");
            socket.close();
            return;
        }
        heartbeatMissed++;
        // envia um ping silencioso — servidor ecoa mensagens de broadcast
        // como fallback, usamos a mensagem regular de status do servidor
    }, HEARTBEAT_INTERVAL);
}

function stopHeartbeat() {
    if (heartbeatTimer) {
        clearInterval(heartbeatTimer);
        heartbeatTimer = null;
    }
}

function scheduleReconnect() {
    if (!reconnectTimer) {
        reconnectTimer = setTimeout(() => {
            reconnectTimer = null;
            initWebSocket();
        }, RECONNECT_INTERVAL);
    }
}

// Reconecta imediatamente ao voltar para a aba (mobile suspend/resume)
document.addEventListener('visibilitychange', () => {
    if (!document.hidden) {
        console.log('Aba voltou ao foco — verificando WebSocket...');
        if (!socket || socket.readyState !== WebSocket.OPEN) {
            initWebSocket();
        }
    }
});

// Também ao retomar conexão de rede (online/offline events)
window.addEventListener('online', () => {
    console.log('Rede online — reconectando...');
    initWebSocket();
});

window.addEventListener('offline', () => {
    console.log('Rede offline');
    if (socket) socket.close();
});

// Fecha limpo ao sair da página
window.addEventListener('beforeunload', () => {
    if (socket) socket.close();
});

function sendCmd(cmdObj) {
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify(cmdObj));
        return true;
    } else {
        pendingCommands.push(cmdObj);
        // Tenta reconectar imediatamente
        initWebSocket();
        return false;
    }
}

function map(val, inMin, inMax, outMin, outMax) {
    return (val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

let isManual = false;
let isSpinning = false;
let currentSpeed = 33;

const modeManual = document.getElementById('modeManual');
const modeAuto = document.getElementById('modeAuto');
const motorToggle = document.getElementById('motorToggle');
const toggleKnob = document.getElementById('toggleKnob');
const motorText = document.getElementById('motorText');
const discGroup = document.getElementById('discGroup');
const tonearmGroup = document.getElementById('tonearmGroup');
const rpm33 = document.getElementById('rpm33');
const rpm45 = document.getElementById('rpm45');
const liftSlider = document.getElementById('liftSlider');    
const liftValue = document.getElementById('liftValue');
const liftUp = document.getElementById('liftUp');
const liftDown = document.getElementById('liftDown');
const advancedBtn = document.getElementById('advancedBtn');

function updateUI(data) {
    // Atualiza estado do modo Operação
    if (modeManual && modeAuto && data.manualOp !== undefined) {
        isManual = data.manualOp;
        updateModeVisualState();
    }

    // Atualizar botão de Power (somente se servidor confirmar estado diferente)
    if (motorToggle && toggleKnob && motorText && discGroup && data.motorLigado !== undefined) {
        if (isSpinning !== data.motorLigado) {
            isSpinning = data.motorLigado;
            updateMotorVisualState();
        }
    }

    // Atualizar botões de RPM (somente se servidor confirmar estado diferente)    
    if (rpm33 && rpm45 && data.rpm !== undefined) {
        const serverSpeed = (data.rpm < 40) ? 33 : 45;
        if (currentSpeed !== serverSpeed) {
            currentSpeed = serverSpeed;
            updateRPMVisualState();
        }
    }

    // Mover o tonearm
    if (tonearmGroup && data.tonearmAngle !== undefined) {
        const visualAngle = map(data.tonearmAngle, 173.8, 125.0, 0, 50.2);
        const clamped = Math.max(Math.min(visualAngle, 30), -70);
        tonearmGroup.style.transform = `rotate(${clamped}deg)`;
    }

    // Atualiza o elemento de exibição do ângulo em tempo real
    const tonearmAngleSpan = document.getElementById('tonearmAngleSpan');
    if (tonearmAngleSpan && data.tonearmAngle !== undefined) {
        tonearmAngleSpan.innerText = data.tonearmAngle.toFixed(1);
    }

    // Atualiza posição do servo (apenas se não estiver sendo modificado pelo usuário ativamente)
    if (liftSlider && liftValue && data.servoPos !== undefined && document.activeElement !== liftSlider) {
        liftValue.textContent = data.servoPos+'°';
        liftSlider.value = data.servoPos;
        if (data.liftMax !== undefined) liftSlider.min = Math.min(data.liftMax, data.liftMin);
        if (data.liftMin !== undefined) liftSlider.max = Math.max(data.liftMax, data.liftMin);
    }
}

function toggleMotor() {
    // Feedback visual imediato antes de confirmar do servidor
    isSpinning = !isSpinning;
    updateMotorVisualState();
    
    sendCmd({ cmd: "toggle" });
}

function setOperMode(manual) {
    isManual = manual;
    updateModeVisualState();
    sendCmd({ cmd: "oper", val: manual ? "manual" : "auto" });
}

function setRPM(val) {
    currentSpeed = (val < 40) ? 33 : 45;
    updateRPMVisualState();
    
    sendCmd({ cmd: "rpm", val: val });
}

function setLift(val) {
    liftValue.textContent = val+'°';
    sendCmd({ cmd: "servo", val: parseFloat(val) });
}

function updateMotorVisualState() {
    if (!motorToggle || !toggleKnob || !motorText || !discGroup) return;
    if (isSpinning) {
        motorToggle.style.background = '#2ecc71';
        toggleKnob.style.transform = 'translateX(28px)';
        motorText.textContent = t('motor.turn_off');
        discGroup.classList.add('spinning');
        updateSpinSpeed();
    } else {
        motorToggle.style.background = '#333';
        toggleKnob.style.transform = 'translateX(0)';
        motorText.textContent = t('motor.turn_on');
        discGroup.classList.remove('spinning');
        updateSpinSpeed();
    }
}

function updateModeVisualState() {
    if (!modeManual || !modeAuto) return;
    if (isManual) {
        modeManual.classList.toggle('active', true);
        modeAuto.classList.toggle('active', false);
    } else {
        modeManual.classList.toggle('active', false);
        modeAuto.classList.toggle('active', true);
    }
}

function updateRPMVisualState() {
    if (!rpm33 || !rpm45) return;
    rpm33.classList.toggle('active', currentSpeed < 40);
    rpm45.classList.toggle('active', currentSpeed >= 40);
}

function updateSpinSpeed() {
// Girar o disco conforme RPM

  if (!isSpinning) return;
  const duration = 60 / currentSpeed;
  discGroup.style.animationDuration = `${duration}s`;
}

window.onload = function() {
    initWebSocket();
    
    if ('serviceWorker' in navigator) {
        navigator.serviceWorker.register('/sw.js')
        .then(reg => console.log('SW registered'))
        .catch(err => console.log('SW registration failed'));
    }

    // modo de operação
    modeManual.addEventListener('click', () => {
        setOperMode(true);
    });
    modeAuto.addEventListener('click', () => {
        setOperMode(false);
    });

    // controle do motor
    motorToggle.addEventListener('click', () => {
        toggleMotor();
    });
    
    // controle de RPM

    rpm33.addEventListener('click', () => setRPM(33));
    rpm45.addEventListener('click', () => setRPM(45));
    liftSlider.addEventListener('input', () => {
        const angle = parseFloat(liftSlider.value);
        
        setLift(angle);
    });

    // controle do lift
    liftUp.addEventListener('click', () => {
        liftUp.classList.toggle('active', true);
        liftSlider.value = liftSlider.min;
        const angle = parseFloat(liftSlider.value);
        setLift(angle);
        liftUp.classList.toggle('active', false);
    });
    liftDown.addEventListener('click', () => {
        liftUp.classList.toggle('active', true);
        liftSlider.value = liftSlider.max;
        const angle = parseFloat(liftSlider.value);
        setLift(angle);
        liftUp.classList.toggle('active', false);
    });

    // configuração avançada
    advancedBtn.addEventListener('click', () => {
        window.location.href = '/config.html';
    });
}