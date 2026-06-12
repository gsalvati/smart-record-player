const translations = {
  en: {
    // index.html
    'motor.section':       'MOTOR & ROTATION',
    'motor.turn_on':       'TURN ON MOTOR',
    'motor.turn_off':      'TURN OFF MOTOR',
    'motor.turn_off_hint': 'TURN OFF MOTOR',
    'mode.label':          'OPERATION MODE',
    'mode.manual':         'MANUAL',
    'mode.auto':           'AUTOMATIC',
    'lift.section':        'LIFT CONTROL',
    'lift.position':       'Current position:',
    'btn.advanced':        '⚙️ ADVANCED SETTINGS',

    // config.html - tabs
    'tab.lift':     'Lift',
    'tab.tonearm':  'Tonearm',
    'tab.timing':   'Timing',
    'tab.files':    'Files',
    'tab.showall':  'Show All',

    // config.html - lift section
    'lift.title':   'Lift (Servo)',
    'lift.hint':    'Use the slider to adjust visually; fine-tune with the number field.',
    'lift.raised':  'Raised (liftMax):',
    'lift.lowered': 'Lowered (liftMin):',

    // config.html - tonearm section
    'tonearm.title': 'Tonearm',
    'tonearm.hint':  'Set the angle limits that control motor on/off.',
    'tonearm.max':   'Max angle (vertical):',
    'tonearm.min':   'Min angle (center):',

    // config.html - timing section
    'timing.title':    'Timing & Debounce',
    'timing.hint':     'Timing controls that affect automatic behavior.',
    'timing.debounce': 'Debounce (s):',
    'timing.rise':     'Rise time (ms):',
    'timing.fall':     'Fall time (ms):',

    // config.html - files section
    'files.title':        'File Manager (LittleFS)',
    'files.loading':      'Loading files...',
    'files.upload_btn':   'UPLOAD FILE',
    'files.none':         'No files found.',
    'files.error':        'Error loading files.',
    'files.uploading':    'Uploading...',
    'files.uploaded':     'Uploaded successfully!',
    'files.upload_error': 'Upload error.',

    // config.html - buttons & heading
    'config.heading': 'Settings - GIRA',
    'btn.save':  'SAVE SETTINGS',
    'btn.back':  'BACK',
    'btn.wifi':  '⚙️ CHANGE WIFI NETWORK',

    // language switcher
    'lang.label': 'Language',

    // wifi.html - scan
    'wifi.networks':      'Available Networks',
    'wifi.scan_refresh':  '↻ Refresh',
    'wifi.scan_loading':  'Scanning networks...',
    'wifi.scan_error':    'Scan failed. Try again.',
    'wifi.scan_none':     'No networks found.',
    'wifi.select_first':  '⚠️ Please select a network.',

    // wifi.html
    'wifi.heading':     'WiFi Configuration',
    'wifi.subtitle':    'Connect the Record Player to your home network.<br>After saving, the device will restart.',
    'wifi.ssid_label':  'Network Name (SSID)',
    'wifi.ssid_ph':     'E.g.: MyNetwork',
    'wifi.pass_label':  'Network Password',
    'wifi.pass_ph':     'WiFi Password',
    'wifi.btn_connect': 'CONNECT AND SAVE',
    'wifi.btn_saving':  'Saving...',
    'wifi.no_ssid':     '⚠️ Please enter the network name (SSID).',
    'wifi.info':        '⏳ Saving credentials and restarting...',
    'wifi.saved':       '✅ Saved! The Record Player is restarting.<br>Reconnect to your WiFi and access <strong>http://gira.local</strong>',
    'wifi.footer':      'After saving, the device will try to connect to the given network.<br>Reconnect your device to your regular WiFi and access<br><strong>http://gira.local</strong>',
  },

  pt: {
    'motor.section':       'MOTOR & ROTAÇÃO',
    'motor.turn_on':       'LIGAR MOTOR',
    'motor.turn_off':      'DESLIGAR MOTOR',
    'motor.turn_off_hint': 'DESLIGAR MOTOR',
    'mode.label':          'MODO DE FUNCIONAMENTO',
    'mode.manual':         'MANUAL',
    'mode.auto':           'AUTOMÁTICO',
    'lift.section':        'CONTROLE DO LIFT',
    'lift.position':       'Posição atual:',
    'btn.advanced':        '⚙️ CONFIGURAÇÕES AVANÇADAS',
    'tab.lift':    'Lift',
    'tab.tonearm': 'Braço',
    'tab.timing':  'Tempos',
    'tab.files':   'Arquivos',
    'tab.showall': 'Mostrar Tudo',
    'lift.title':   'Lift (Servo)',
    'lift.hint':    'Use o slider para ajustar visualmente; ajuste fino com o campo numérico.',
    'lift.raised':  'Levantado (liftMax):',
    'lift.lowered': 'Baixado (liftMin):',
    'tonearm.title': 'Braço (Tonearm)',
    'tonearm.hint':  'Defina os limites de ângulo que controlam ligar/desligar o motor.',
    'tonearm.max':   'Ângulo Máx (vertical):',
    'tonearm.min':   'Ângulo Mín (centro):',
    'timing.title':    'Tempos & Debounce',
    'timing.hint':     'Controles de temporização que afetam o comportamento automático.',
    'timing.debounce': 'Debounce (s):',
    'timing.rise':     'Tempo de subida (ms):',
    'timing.fall':     'Tempo de descida (ms):',
    'files.title':        'Gerenciador de Arquivos (LittleFS)',
    'files.loading':      'Carregando arquivos...',
    'files.upload_btn':   'ENVIAR ARQUIVO',
    'files.none':         'Nenhum arquivo encontrado.',
    'files.error':        'Erro ao carregar arquivos.',
    'files.uploading':    'Enviando...',
    'files.uploaded':     'Enviado com sucesso!',
    'files.upload_error': 'Erro ao enviar.',
    'config.heading': 'Configurações - GIRA',
    'btn.save': 'SALVAR CONFIGURAÇÕES',
    'btn.back': 'VOLTAR',
    'btn.wifi': '⚙️ ALTERAR REDE WIFI',
    'lang.label': 'Idioma',

    'wifi.networks':      'Redes Disponíveis',
    'wifi.scan_refresh':  '↻ Atualizar',
    'wifi.scan_loading':  'Buscando redes...',
    'wifi.scan_error':    'Falha no scan. Tente novamente.',
    'wifi.scan_none':     'Nenhuma rede encontrada.',
    'wifi.select_first':  '⚠️ Por favor selecione uma rede.',
    'wifi.heading':     'Configuração WiFi',
    'wifi.subtitle':    'Conecte o Toca-Discos à sua rede doméstica.<br>Após salvar, o dispositivo irá reiniciar.',
    'wifi.ssid_label':  'Nome da Rede (SSID)',
    'wifi.ssid_ph':     'Ex: MinhaRede',
    'wifi.pass_label':  'Senha da Rede',
    'wifi.pass_ph':     'Senha do WiFi',
    'wifi.btn_connect': 'CONECTAR E SALVAR',
    'wifi.btn_saving':  'Salvando...',
    'wifi.no_ssid':     '⚠️ Por favor informe o nome da rede (SSID).',
    'wifi.info':        '⏳ Salvando credenciais e reiniciando...',
    'wifi.saved':       '✅ Salvo! O Toca-Discos está reiniciando.<br>Reconecte à sua rede WiFi e acesse <strong>http://gira.local</strong>',
    'wifi.footer':      'Após salvar, o aparelho tentará conectar à rede informada.<br>Reconecte seu dispositivo à sua rede WiFi normal e acesse<br><strong>http://gira.local</strong>',
  },

  es: {
    'motor.section':       'MOTOR & ROTACIÓN',
    'motor.turn_on':       'ENCENDER MOTOR',
    'motor.turn_off':      'APAGAR MOTOR',
    'motor.turn_off_hint': 'APAGAR MOTOR',
    'mode.label':          'MODO DE OPERACIÓN',
    'mode.manual':         'MANUAL',
    'mode.auto':           'AUTOMÁTICO',
    'lift.section':        'CONTROL DEL LIFT',
    'lift.position':       'Posición actual:',
    'btn.advanced':        '⚙️ AJUSTES AVANZADOS',
    'tab.lift':    'Lift',
    'tab.tonearm': 'Brazo',
    'tab.timing':  'Tiempos',
    'tab.files':   'Archivos',
    'tab.showall': 'Mostrar Todo',
    'lift.title':   'Lift (Servo)',
    'lift.hint':    'Use el control deslizante para ajustar visualmente; ajuste fino con el campo numérico.',
    'lift.raised':  'Levantado (liftMax):',
    'lift.lowered': 'Bajado (liftMin):',
    'tonearm.title': 'Brazo (Tonearm)',
    'tonearm.hint':  'Defina los límites de ángulo que controlan encender/apagar el motor.',
    'tonearm.max':   'Ángulo Máx (vertical):',
    'tonearm.min':   'Ángulo Mín (centro):',
    'timing.title':    'Tiempos & Debounce',
    'timing.hint':     'Controles de temporización que afectan el comportamiento automático.',
    'timing.debounce': 'Debounce (s):',
    'timing.rise':     'Tiempo de subida (ms):',
    'timing.fall':     'Tiempo de bajada (ms):',
    'files.title':        'Gestor de Archivos (LittleFS)',
    'files.loading':      'Cargando archivos...',
    'files.upload_btn':   'SUBIR ARCHIVO',
    'files.none':         'No se encontraron archivos.',
    'files.error':        'Error al cargar archivos.',
    'files.uploading':    'Subiendo...',
    'files.uploaded':     '¡Subido con éxito!',
    'files.upload_error': 'Error al subir.',
    'config.heading': 'Configuración - GIRA',
    'btn.save': 'GUARDAR AJUSTES',
    'btn.back': 'VOLVER',
    'btn.wifi': '⚙️ CAMBIAR RED WIFI',
    'lang.label': 'Idioma',
    'wifi.networks':      'Redes Disponibles',
    'wifi.scan_refresh':  '↻ Actualizar',
    'wifi.scan_loading':  'Buscando redes...',
    'wifi.scan_error':    'Error en el scan. Intente de nuevo.',
    'wifi.scan_none':     'No se encontraron redes.',
    'wifi.select_first':  '⚠️ Por favor seleccione una red.',
    'wifi.heading':     'Configuración WiFi',
    'wifi.subtitle':    'Conecte el Toca-Discos a su red doméstica.<br>Tras guardar, el dispositivo se reiniciará.',
    'wifi.ssid_label':  'Nombre de Red (SSID)',
    'wifi.ssid_ph':     'Ej: MiRed',
    'wifi.pass_label':  'Contraseña de Red',
    'wifi.pass_ph':     'Contraseña WiFi',
    'wifi.btn_connect': 'CONECTAR Y GUARDAR',
    'wifi.btn_saving':  'Guardando...',
    'wifi.no_ssid':     '⚠️ Por favor ingrese el nombre de la red (SSID).',
    'wifi.info':        '⏳ Guardando credenciales y reiniciando...',
    'wifi.saved':       '✅ ¡Guardado! El Toca-Discos está reiniciando.<br>Reconéctese a su WiFi y acceda a <strong>http://gira.local</strong>',
    'wifi.footer':      'Tras guardar, el dispositivo intentará conectarse a la red indicada.<br>Reconecte su dispositivo a su WiFi habitual y acceda a<br><strong>http://gira.local</strong>',
  },
};

function getLang() {
  return localStorage.getItem('gira_lang') || 'en';
}

function setLang(lang) {
  localStorage.setItem('gira_lang', lang);
  applyTranslations(lang);
  updateLangButtons(lang);
}

function t(key) {
  const lang = getLang();
  return (translations[lang] && translations[lang][key] !== undefined)
    ? translations[lang][key]
    : (translations['en'][key] !== undefined ? translations['en'][key] : key);
}

function applyTranslations(lang) {
  const tr = translations[lang] || translations['en'];
  document.querySelectorAll('[data-i18n]').forEach(el => {
    const key = el.dataset.i18n;
    if (tr[key] !== undefined) el.textContent = tr[key];
  });
  document.querySelectorAll('[data-i18n-html]').forEach(el => {
    const key = el.dataset.i18nHtml;
    if (tr[key] !== undefined) el.innerHTML = tr[key];
  });
  document.querySelectorAll('[data-i18n-placeholder]').forEach(el => {
    const key = el.dataset.i18nPlaceholder;
    if (tr[key] !== undefined) el.placeholder = tr[key];
  });
  const titleEl = document.querySelector('title[data-i18n]');
  if (titleEl && tr[titleEl.dataset.i18n]) document.title = tr[titleEl.dataset.i18n];
  if (document.documentElement) document.documentElement.lang = lang;
}

function updateLangButtons(lang) {
  document.querySelectorAll('.lang-btn').forEach(btn => {
    const active = btn.dataset.lang === lang;
    btn.style.opacity     = active ? '1' : '0.4';
    btn.style.fontWeight  = active ? '700' : '400';
    btn.style.borderColor = active ? '#ff6a00' : '#333';
    btn.style.color       = active ? '#ff6a00' : '#aaa';
  });
}

document.addEventListener('DOMContentLoaded', function () {
  const lang = getLang();
  applyTranslations(lang);
  updateLangButtons(lang);
});
