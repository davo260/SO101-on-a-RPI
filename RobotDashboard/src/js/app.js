document.addEventListener('DOMContentLoaded', () => {
  const targetViewer = createArmViewer(document.getElementById('targetContainer'), 'target');
  const realViewer = createArmViewer(document.getElementById('realContainer'), 'real');

  const logsFeed = document.getElementById('logsFeed');
  const btnClearLogs = document.getElementById('btnClearLogs');
  const statusBadge = document.getElementById('netStatus');
  const btnMqttToggle = document.getElementById('btnMqttToggle');
  const pillMode = document.getElementById('robotMode');
  const pillFaults = document.getElementById('robotFaults');
  const btnEmergency = document.getElementById('btnEmergency');
  const modeButtons = document.querySelectorAll('.mode-btn');
  const ctrlButtons = document.querySelectorAll('.ctrl-btn');
  const btnHome = document.getElementById('btnHome');
  const targetTag = document.querySelector('#targetContainer .canvas-tag');

  let isMqttConnected = false;
  let isEStopLatched = false;

  // ===== Modo de control: 'local' (por defecto) | 'dashboard' =====
  //  local     -> el usuario mueve el líder a mano. El dashboard NO envía consignas; los selectores y el
  //               modelo de consigna siguen la posición real del líder.
  //  dashboard -> los selectores comandan el líder en tiempo real. Mientras se mueve un selector se ignora
  //               la posición del líder para esa articulación (para no pelear con el usuario).
  const SEND_INTERVAL_MS = 60;   // ráfaga de consignas (~16 Hz)
  const HOLD_MS = 800;           // tras tocar un selector, no se sobrescribe con la posición del líder
  let controlMode = 'local';
  let pendingControl = null;
  let bridgeReportsControl = false;
  let statesWithoutLeader = 0;
  const lastUserInput = {};
  const draggingJoints = new Set();
  const pendingJoints = new Set();
  let sendTimer = null;

  // Calibración y Rangos Crudos (SO-101)
  const CALIBRATION = {
    shoulder_pan:  { LnormMin: -100, LrawMin: 729,  LnormMax: 100, LrawMax: 4000 },
    shoulder_lift: { LnormMin: -100, LrawMin: 1822, LnormMax: 100, LrawMax: 3323 },
    elbow_flex:    { LnormMin: -100, LrawMin: 824,  LnormMax: 100, LrawMax: 3051 },
    wrist_flex:    { LnormMin: -100, LrawMin: 896,  LnormMax: 100, LrawMax: 2800 },
    wrist_roll:    { LnormMin: -100, LrawMin: 0,    LnormMax: 100, LrawMax: 4095 },
    gripper:       { LnormMin: 0,    LrawMin: 2040, LnormMax: 100, LrawMax: 3400 }
  };

  // Posición inicial neutra de la consigna (Lnorm)
  const targetValues = {
    shoulder_pan: 0,
    shoulder_lift: 0,
    elbow_flex: 0,
    wrist_flex: 0,
    wrist_roll: 0,
    gripper: 0
  };

  /**
   * Conversión exacta y lineal Lnorm -> Lraw (Ticks 0..4095)
   */
  function valueToRaw(joint, val) {
    const c = CALIBRATION[joint];
    if (!c) return Math.round(val);

    const v = parseFloat(val);
    const clampedVal = Math.min(Math.max(v, c.LnormMin), c.LnormMax);

    const spanNorm = c.LnormMax - c.LnormMin;
    const factor = (clampedVal - c.LnormMin) / spanNorm;

    const spanRaw = c.LrawMax - c.LrawMin;
    const rawResult = Math.round(c.LrawMin + factor * spanRaw);

    // Clamping estricto dentro de los límites de hardware
    return Math.min(Math.max(rawResult, c.LrawMin), c.LrawMax);
  }

  targetViewer.setNormalizedPose(targetValues);

  function setStatusBadge(statusText, color = '#38ef7d') {
    statusBadge.innerHTML = `<span class="dot" style="background:${color};box-shadow:0 0 8px ${color};"></span> ${statusText}`;
    statusBadge.style.color = color;
  }

  function updateMqttButton(connected) {
    isMqttConnected = connected;
    if (connected) {
      btnMqttToggle.textContent = 'Desconectar MQTT';
      btnMqttToggle.className = 'btn btn-mqtt-disconnect';
    } else {
      btnMqttToggle.textContent = 'Conectar MQTT';
      btnMqttToggle.className = 'btn btn-mqtt-connect';
    }
  }

  // Resalta el botón del modo que informa el demonio
  function updateModeButtons(mode) {
    const m = mode === 'teleoperador' ? 'teleop' : mode;
    modeButtons.forEach((b) => b.classList.toggle('active', b.dataset.mode === m));
  }

  function appendLog(level, message, detail = null, timeStr = null) {
    const time = timeStr || new Date().toLocaleTimeString();
    const entry = document.createElement('div');
    entry.className = `log-entry log-${level}`;

    entry.innerHTML = `
      <span class="log-time">[${time}]</span>
      <span class="log-badge">${level}</span>
      <span class="log-msg">${message}</span>
      ${detail ? `<span class="log-detail">↳ ${detail}</span>` : ''}
    `;

    logsFeed.appendChild(entry);
    logsFeed.scrollTop = logsFeed.scrollHeight;
  }

  btnClearLogs.addEventListener('click', () => {
    logsFeed.innerHTML = '';
  });

  // Conexión WebSocket con el servidor backend
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${protocol}//${window.location.host}`;
  const socket = new WebSocket(wsUrl);

  socket.onopen = () => {
    appendLog('info', 'WebSocket conectado con el backend.');
  };

  socket.onclose = () => {
    setStatusBadge('Backend desconectado', '#ef4444');
    appendLog('error', 'Desconectado del servidor local.');
    applyControlMode('local');
    updateMqttButton(false);
  };

  btnMqttToggle.addEventListener('click', () => {
    if (socket.readyState !== WebSocket.OPEN) return;

    if (!isMqttConnected) {
      socket.send(JSON.stringify({ type: 'mqtt_connect' }));
      setStatusBadge('Conectando...', '#facc15');
    } else {
      socket.send(JSON.stringify({ type: 'mqtt_disconnect' }));
    }
  });

  socket.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);

      if (msg.type === 'mqtt_status_change') {
        if (msg.status === 'connected') {
          setStatusBadge('MQTT Conectado', '#38ef7d');
          updateMqttButton(true);
        } else if (msg.status === 'connecting') {
          setStatusBadge('Conectando...', '#facc15');
        } else if (msg.status === 'disconnected') {
          setStatusBadge('MQTT Desconectado', '#64748b');
          updateMqttButton(false);
        } else if (msg.status === 'error') {
          setStatusBadge('Error MQTT', '#ef4444');
          updateMqttButton(false);
        }
      } else if (msg.type === 'robot_state') {
        handleRobotState(msg.data);
      } else if (msg.type === 'log_event') {
        appendLog(msg.level, msg.message, msg.detail, msg.timestamp);
      }
    } catch (e) {
      appendLog('error', 'Error parseando mensaje WebSocket', e.message);
    }
  };

  // Manejo de la telemetría recibida desde so101/state
  function handleRobotState(state) {
    // 0. Modo de control informado por el puente (es la fuente de verdad)
    if (state.control_mode) {
      bridgeReportsControl = true;
      if (state.control_mode !== controlMode) applyControlMode(state.control_mode);
      pendingControl = null;
    }

    // 1. Modos de Operación (hold, teleoperador, estop, idle, reset)
    if (state.mode) {
      const m = state.mode.toLowerCase().trim();
      pillMode.textContent = `MODO: ${m.toUpperCase()}`;
      pillMode.className = 'pill pill-mode';

      switch (m) {
        case 'hold':
          pillMode.classList.add('mode-hold');
          break;
        case 'teleoperador':
        case 'teleop':
          pillMode.classList.add('mode-teleoperador');
          break;
        case 'estop':
          pillMode.classList.add('mode-estop');
          setEmergencyButtonState(true);
          break;
        case 'idle':
          pillMode.classList.add('mode-idle');
          break;
        case 'reset':
          pillMode.classList.add('mode-reset');
          break;
        default:
          pillMode.classList.add('mode-idle');
          break;
      }
      updateModeButtons(m);
    }

    // 2. Fallas
    if (state.faults !== undefined) {
      const hasFaults = Array.isArray(state.faults) ? state.faults.length > 0 : !!state.faults;
      if (hasFaults) {
        pillFaults.textContent = `Fallas: ${Array.isArray(state.faults) ? state.faults.join(', ') : state.faults}`;
        pillFaults.classList.add('has-fault');
      } else {
        pillFaults.textContent = 'Fallas: Ninguna';
        pillFaults.classList.remove('has-fault');
      }
    }

    // 3. Posición real recibida (ya viene en Lnorm desde la Raspberry)
    if (state.position) {
      // Articulaciones sin dato (null) se omiten para no romper el visor 3D
      const pose = {};
      Object.entries(state.position).forEach(([j, v]) => {
        if (v !== null && v !== undefined) pose[j] = v;
      });
      realViewer.setNormalizedPose(pose);
    }

    // 3b. Posición del LÍDER: mueve los selectores y el modelo de consigna
    if (state.leader_position) {
      statesWithoutLeader = 0;
      applyLeaderPosition(state.leader_position);
    } else if (++statesWithoutLeader === 30) {
      appendLog('warn', 'El puente no envía leader_position: los selectores no seguirán al líder.',
        'Activa READ_LEADER en el .ini del puente (V8).');
    }

    // 4. Telemetría de Servos: Solo Temperatura y Voltaje
    if (state.telemetry) {
      Object.keys(state.telemetry).forEach((joint) => {
        const t = state.telemetry[joint];
        if (t.temp !== undefined && t.temp !== null) {
          const el = document.getElementById(`temp-${joint}`);
          if (el) el.textContent = `${Number(t.temp).toFixed(1)} °C`;
        }
        if (t.volt !== undefined && t.volt !== null) {
          const el = document.getElementById(`volt-${joint}`);
          if (el) el.textContent = `${Number(t.volt).toFixed(1)} V`;
        }
      });
    }

    // 5. El botón de emergencia refleja el estado real (modo estop o parada del puente)
    const faultList = Array.isArray(state.faults) ? state.faults : [];
    setEmergencyButtonState(
      (state.mode || '').toLowerCase() === 'estop' || faultList.includes('EMERGENCY_STOP_TRIGGERED')
    );
  }

  // ---------- Utilidades de selectores ----------
  function setSliderValue(joint, val) {
    const slider = document.getElementById(`slider-${joint}`);
    const display = document.getElementById(`val-${joint}`);
    const rounded = Math.round(val);
    if (slider) slider.value = rounded;
    if (display) display.textContent = joint === 'gripper' ? `${rounded}%` : `${rounded}`;
  }

  // Los selectores y el modelo de consigna siguen la posición real del líder
  function applyLeaderPosition(leader) {
    const now = Date.now();
    let changed = false;
    Object.keys(targetValues).forEach((j) => {
      const v = Number(leader[j]);
      if (leader[j] === null || leader[j] === undefined || Number.isNaN(v)) return;
      if (controlMode === 'dashboard' &&
          (draggingJoints.has(j) || pendingJoints.has(j) || now - (lastUserInput[j] || 0) < HOLD_MS)) {
        return; // el usuario está moviendo este selector
      }
      targetValues[j] = v;
      setSliderValue(j, v);
      changed = true;
    });
    if (changed) targetViewer.setNormalizedPose(targetValues);
  }

  // ---------- Modo de control ----------
  function applyControlMode(mode) {
    const changed = mode !== controlMode;
    controlMode = mode;
    const local = mode === 'local';
    ctrlButtons.forEach((b) => b.classList.toggle('active', b.dataset.control === mode));
    Object.keys(targetValues).forEach((j) => {
      const s = document.getElementById(`slider-${j}`);
      if (s) s.disabled = local;
    });
    btnHome.disabled = local;
    if (targetTag) targetTag.textContent = local ? 'Brazo Líder (posición actual)' : 'Consigna (Setpoint)';
    if (local) {
      pendingJoints.clear();
      draggingJoints.clear();
      if (sendTimer) { clearTimeout(sendTimer); sendTimer = null; }
    }
    if (changed) {
      appendLog(local ? 'info' : 'warn',
        local ? 'Modo Local: el usuario mueve el líder; el dashboard solo observa.'
              : 'Modo Dashboard: los selectores comandan el brazo líder en tiempo real.');
    }
  }
  applyControlMode('local');

  function canSend() {
    return controlMode === 'dashboard' && isMqttConnected && !isEStopLatched &&
           socket.readyState === WebSocket.OPEN;
  }

  // Envío en ráfaga: agrupa lo movido en SEND_INTERVAL_MS y manda SOLO esas articulaciones
  function queueJoint(joint) {
    pendingJoints.add(joint);
    if (!sendTimer) sendTimer = setTimeout(flushPending, SEND_INTERVAL_MS);
  }

  function flushPending() {
    sendTimer = null;
    if (!canSend()) { pendingJoints.clear(); return; }
    const pos = {};
    const raw = {};
    pendingJoints.forEach((j) => {
      pos[j] = targetValues[j];
      raw[j] = valueToRaw(j, targetValues[j]);
    });
    pendingJoints.clear();
    if (!Object.keys(pos).length) return;
    socket.send(JSON.stringify({
      type: 'publish_motion',
      payload: { command: 'MOVE_JOINTS', stream: true, timestamp: Date.now(), target_position: pos, target_raw: raw }
    }));
  }

  ctrlButtons.forEach((btn) => {
    btn.addEventListener('click', () => {
      const target = btn.dataset.control;
      if (target === controlMode) return;
      if (socket.readyState !== WebSocket.OPEN || !isMqttConnected) {
        appendLog('error', 'Conecta primero el MQTT para cambiar el modo de control.');
        return;
      }

      if (target === 'local') {
        // Salir de Dashboard es la dirección segura: se aplica de inmediato.
        socket.send(JSON.stringify({
          type: 'publish_motion',
          payload: { command: 'SET_CONTROL', control: 'local', timestamp: Date.now() }
        }));
        applyControlMode('local');
        return;
      }

      if (isEStopLatched) {
        appendLog('error', 'El robot está en EMERGENCY STOP. Realiza el rearme primero.');
        return;
      }
      if (!window.confirm('¿Pasar a Modo Dashboard? Los selectores moverán el brazo líder en tiempo real.')) return;

      socket.send(JSON.stringify({
        type: 'publish_motion',
        payload: { command: 'SET_CONTROL', control: 'dashboard', timestamp: Date.now() }
      }));
      if (!bridgeReportsControl) {
        appendLog('warn', 'El puente no informa control_mode (¿versión anterior a V8?).',
          'El bloqueo de consignas se aplica solo en el dashboard.');
        applyControlMode('dashboard');
        return;
      }
      // Entrar a Dashboard solo se aplica cuando el puente lo confirma en so101/state
      pendingControl = 'dashboard';
      setTimeout(() => {
        if (pendingControl === 'dashboard') {
          pendingControl = null;
          appendLog('error', 'El puente no aceptó el Modo Dashboard.',
            'Revisa si hay parada de emergencia activa o si el puente es V8.');
        }
      }, 2000);
    });
  });

  // ---------- Sliders de consigna ----------
  Object.keys(targetValues).forEach((joint) => {
    const slider = document.getElementById(`slider-${joint}`);
    const display = document.getElementById(`val-${joint}`);
    const card = document.querySelector(`.joint-card[data-joint="${joint}"]`);

    slider.addEventListener('input', (e) => {
      if (controlMode !== 'dashboard') return; // en Modo Local los selectores no comandan nada
      const val = parseFloat(e.target.value);
      targetValues[joint] = val;
      display.textContent = joint === 'gripper' ? `${val}%` : `${val}`;
      lastUserInput[joint] = Date.now();

      targetViewer.setNormalizedPose(targetValues);
      targetViewer.highlightJoint(joint);

      document.querySelectorAll('.joint-card').forEach((c) => c.classList.remove('active-motion'));
      card.classList.add('active-motion');

      queueJoint(joint);
    });

    slider.addEventListener('pointerdown', () => draggingJoints.add(joint));
    ['pointerup', 'pointercancel', 'blur'].forEach((ev) => {
      slider.addEventListener(ev, () => {
        draggingJoints.delete(joint);
        lastUserInput[joint] = Date.now();
      });
    });
  });

  // Botón Home Position: solo en Modo Dashboard (mueve el líder a 0)
  btnHome.addEventListener('click', () => {
    if (controlMode !== 'dashboard') {
      appendLog('warn', 'Home solo está disponible en Modo Dashboard.');
      return;
    }
    if (!window.confirm('El brazo líder se moverá a la posición Home. ¿Continuar?')) return;

    const now = Date.now();
    Object.keys(targetValues).forEach((j) => {
      targetValues[j] = 0;
      setSliderValue(j, 0);
      lastUserInput[j] = now;
      queueJoint(j);
    });
    targetViewer.setNormalizedPose(targetValues);
    appendLog('warn', 'Posición Home enviada al líder.');
  });

  // Botones de modo de operación -> SET_MODE (el puente ejecuta "so101ctl <modo>")
  modeButtons.forEach((btn) => {
    btn.addEventListener('click', () => {
      const mode = btn.dataset.mode;
      if (socket.readyState !== WebSocket.OPEN || !isMqttConnected) {
        appendLog('error', 'No hay conexión activa con el servidor o broker.');
        return;
      }
      if (mode === 'teleop' &&
          !window.confirm('¿Activar TELEOP? El seguidor empezará a copiar los movimientos del líder.')) {
        return;
      }
      socket.send(JSON.stringify({
        type: 'publish_motion',
        payload: { command: 'SET_MODE', mode, timestamp: Date.now() }
      }));
      appendLog(mode === 'estop' ? 'error' : 'info', `Solicitando modo ${mode.toUpperCase()} (so101ctl ${mode})...`);
    });
  });

  // Control de Emergency Stop con Rearme
  function setEmergencyButtonState(latched) {
    isEStopLatched = latched;
    if (latched) {
      btnEmergency.textContent = 'REARMAR SISTEMA';
      btnEmergency.className = 'btn btn-danger is-reset';
    } else {
      btnEmergency.textContent = 'EMERGENCY STOP';
      btnEmergency.className = 'btn btn-danger';
    }
  }

  btnEmergency.addEventListener('click', () => {
    if (socket.readyState !== WebSocket.OPEN || !isMqttConnected) {
      appendLog('error', 'No hay conexión activa con el servidor o broker.');
      return;
    }

    if (!isEStopLatched) {
      socket.send(JSON.stringify({
        type: 'publish_motion',
        payload: { command: 'EMERGENCY_STOP', timestamp: Date.now() }
      }));
      setEmergencyButtonState(true);
      appendLog('error', '¡PARADA DE EMERGENCIA ENVIADA A so101/cmd!');
    } else {
      socket.send(JSON.stringify({
        type: 'publish_motion',
        payload: { command: 'RESET_ESTOP', timestamp: Date.now() }
      }));
      setEmergencyButtonState(false);
      appendLog('success', 'Comando de rearme enviado a so101/cmd. Sistema restablecido.');
    }
  });
});