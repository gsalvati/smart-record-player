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

function updateUI(data) {
    // Atualizar botão de Power
    const btnToggle = document.getElementById("btnToggle");
    if (btnToggle) {
        if (data.motorLigado) {
            btnToggle.classList.add("on");
            btnToggle.innerText = "DESLIGAR MOTOR";
        } else {
            btnToggle.classList.remove("on");
            btnToggle.innerText = "LIGAR MOTOR";
        }
    }

    // Atualizar botões de RPM
    const btn33 = document.getElementById("btn33");
    const btn45 = document.getElementById("btn45");
    if (btn33 && btn45) {
        btn33.classList.toggle("active-rpm", data.rpm < 40);
        btn45.classList.toggle("active-rpm", data.rpm >= 40);
    }

    // Girar o disco conforme RPM
    const platter = document.getElementById('platter');
    if (platter) {
        const speed = data.rpm > 40 ? 1.333 : 1.8;
        platter.style.animationDuration = speed + 's';
        platter.style.animationPlayState = data.motorLigado ? 'running' : 'paused';
    }

    // Mover o tonearm
    const tonearm = document.getElementById('tonearm');
    if (tonearm && data.tonearmAngle !== undefined) {
        const visualAngle = map(data.tonearmAngle, 179, 125, -70, -20);
        tonearm.setAttribute('transform', `rotate(${visualAngle} 445 155)`);
    }

    // Atualiza posição do servo (apenas se não estiver sendo modificado pelo usuário ativamente)
    const posSpan = document.getElementById('pos');
    const slider = document.getElementById('servoSlider');
    if (slider && posSpan && data.servoPos !== undefined && document.activeElement !== slider) {
        posSpan.innerText = data.servoPos;
        slider.value = data.servoPos;
        if (data.liftMax !== undefined) slider.min = Math.min(data.liftMax, data.liftMin);
        if (data.liftMin !== undefined) slider.max = Math.max(data.liftMax, data.liftMin);
    }
}

function toggleMotor() {
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({ cmd: "toggle" }));
    }
}

function setRPM(val) {
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({ cmd: "rpm", val: val }));
    }
}

function setLift(val) {
    document.getElementById('pos').innerText = val;
    if (socket && socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({ cmd: "servo", val: parseFloat(val) }));
    }
}

window.onload = function() {
    initWebSocket();
    
    if ('serviceWorker' in navigator) {
        navigator.serviceWorker.register('/sw.js')
        .then(reg => console.log('SW registered'))
        .catch(err => console.log('SW registration failed'));
    }
};
