let socket;

function initWebSocket() {
    socket = new WebSocket(`ws://${window.location.hostname}:81/`);
    
    socket.onopen = function(e) {
        console.log("WebSocket conectado");
    };
    
    socket.onmessage = function(event) {
        const data = JSON.parse(event.data);
        updateUI(data);
    };
    
    socket.onclose = function(event) {
        console.log("WebSocket desconectado, tentando novamente em 2s...");
        setTimeout(initWebSocket, 2000);
    };
    
    socket.onerror = function(error) {
        console.log("WebSocket erro: ", error);
    };
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
    // Atualiza estado do botão Operação
    
    if (modeManual && modeAuto && data.manualOp !== undefined) {
        isManual = data.manualOp;
        if (isManual) {
            modeManual.classList.toggle('active', true);
            modeAuto.classList.toggle('active', false);
        } else {
            modeManual.classList.toggle('active', false);
            modeAuto.classList.toggle('active', true);
        }
    }

    // Atualizar botão de Power
    if (motorToggle && toggleKnob && motorText && discGroup) {
        if (data.motorLigado) {
            isSpinning = true;
            motorToggle.style.background = '#2ecc71';
            toggleKnob.style.transform = 'translateX(28px)';
            motorText.textContent = 'DESLIGAR MOTOR';
            discGroup.classList.add('spinning');
            updateSpinSpeed();
        } else {
            isSpinning = false;
            motorToggle.style.background = '#333';
            toggleKnob.style.transform = 'translateX(0)';
              motorText.textContent = 'LIGAR MOTOR';
            discGroup.classList.remove('spinning');
            updateSpinSpeed();
        }
    }

    // Atualizar botões de RPM    
    if (rpm33 && rpm45) {
        currentSpeed = data.rpm;
        rpm33.classList.toggle('active', data.rpm < 40);
        rpm45.classList.toggle('active', data.rpm >= 40);        
    }

    // Mover o tonearm
    //const tonearmAngleSpan = document.getElementById('angleValue');
    //if (tonearmAngleSpan && data.tonearmAngle !== undefined) {
     //   tonearmAngleSpan.innerText = data.tonearmAngle.toFixed(1);
    //}
    if (tonearmGroup && data.tonearmAngle !== undefined) {
        const visualAngle = map(data.tonearmAngle, 179, 125, -70, -20);

        //tonearm.setAttribute('transform', `rotate(${visualAngle} 445 155)`);
        tonearmGroup.style.transform = `rotate(${visualAngle}deg)`;
    }

    // Atualiza posição do servo (apenas se não estiver sendo modificado pelo usuário ativamente)
    //const posSpan = document.getElementById('pos');
    //const slider = document.getElementById('servoSlider');
    if (liftSlider && liftValue && data.servoPos !== undefined && document.activeElement !== liftSlider) {
        //posSpan.innerText = data.servoPos;
        liftValue.textContent = data.servoPos+'°';
        liftSlider.value = data.servoPos;
        if (data.liftMax !== undefined) liftSlider.min = Math.min(data.liftMax, data.liftMin);
        if (data.liftMin !== undefined) liftSlider.max = Math.max(data.liftMax, data.liftMin);
    }
}

function toggleMotor() {
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({ cmd: "toggle" }));
    }
}

function toggleOper() {
    if (socket && socket.readyState === WebSocket.OPEN) {
        const newMode = isManual ? "manual" : "auto";
        socket.send(JSON.stringify({ cmd: "oper", val: newMode }));
    }
}

function setRPM(val) {
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({ cmd: "rpm", val: val }));
    }
}

function setLift(val) {
    liftValue.textContent = val+'°';
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({ cmd: "servo", val: parseFloat(val) }));
    }
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
        isManual = true;
        toggleOper();
    });
    modeAuto.addEventListener('click', () => {
        isManual = false;
        toggleOper();
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