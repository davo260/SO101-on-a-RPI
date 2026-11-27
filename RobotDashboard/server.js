require('dotenv').config();
const express = require('express');
const http = require('http');
const path = require('path');
const mqtt = require('mqtt');
const WebSocket = require('ws');

const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

const PORT = process.env.PORT || 3000;
const MQTT_BROKER_URL = process.env.MQTT_BROKER || 'mqtt://localhost:1883';
const MQTT_USER = process.env.MQTT_USER || 'dashboard';
const MQTT_PASSWORD = process.env.MQTT_PASSWORD || 'Dash044#';

// Nueva arquitectura de tópicos SO101
const TOPIC_CMD = 'so101/cmd';
const TOPIC_STATE = 'so101/state';
const TOPIC_EVENT = 'so101/event';
const TOPIC_RESULT = 'so101/cmd/result';

let mqttClient = null;

app.use(express.json());
app.use(express.static(path.join(__dirname, 'src')));

function broadcastLog(level, message, detail = null) {
  const packet = JSON.stringify({
    type: 'log_event',
    timestamp: new Date().toLocaleTimeString(),
    level,
    message,
    detail
  });

  wss.clients.forEach((client) => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(packet);
    }
  });
}

function broadcastMqttStatus(status) {
  const packet = JSON.stringify({
    type: 'mqtt_status_change',
    status
  });

  wss.clients.forEach((client) => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(packet);
    }
  });
}

function connectMQTT() {
  if (mqttClient && (mqttClient.connected || mqttClient.reconnecting)) {
    broadcastLog('warn', 'Ya existe una conexión MQTT activa.');
    return;
  }

  broadcastLog('info', `Conectando con usuario '${MQTT_USER}' a: ${MQTT_BROKER_URL}`);
  broadcastMqttStatus('connecting');

  const options = {
    reconnectPeriod: 3000,
    clientId: 'dashboard_ui_' + Math.random().toString(16).substring(2, 8),
    username: MQTT_USER,
    password: MQTT_PASSWORD
  };

  mqttClient = mqtt.connect(MQTT_BROKER_URL, options);

  mqttClient.on('connect', () => {
    broadcastLog('success', `Conectado al broker como '${MQTT_USER}'`);
    broadcastMqttStatus('connected');

    // Suscribirse a los 3 tópicos de lectura definidos en el ACL
    const topicsToSub = [TOPIC_STATE, TOPIC_EVENT, TOPIC_RESULT];
    mqttClient.subscribe(topicsToSub, (err) => {
      if (err) {
        broadcastLog('error', 'Fallo al suscribirse a los tópicos so101', err.message);
      } else {
        broadcastLog('info', `Suscripción exitosa a: ${topicsToSub.join(', ')}`);
      }
    });
  });

  mqttClient.on('message', (topic, payload) => {
    const rawData = payload.toString();
    try {
      const parsed = JSON.parse(rawData);

      // Reenvío al cliente web según el tópico
      if (topic === TOPIC_STATE) {
        wss.clients.forEach((c) => {
          if (c.readyState === WebSocket.OPEN) {
            c.send(JSON.stringify({ type: 'robot_state', data: parsed }));
          }
        });
      } else if (topic === TOPIC_EVENT) {
        broadcastLog('warn', `[EVENTO ROBOT] Modo: ${parsed.mode || rawData}`, JSON.stringify(parsed));
      } else if (topic === TOPIC_RESULT) {
        const lvl = parsed.status === 'OK' ? 'success' : 'error';
        broadcastLog(lvl, `[RESPUESTA CMD] Estado: ${parsed.status}`, parsed.detail || null);
      }
    } catch (e) {
      // Manejo de mensajes de texto plano en event o result
      if (topic === TOPIC_EVENT) {
        broadcastLog('warn', `[EVENTO ROBOT]: ${rawData}`);
      } else if (topic === TOPIC_RESULT) {
        const lvl = rawData.includes('OK') ? 'success' : 'error';
        broadcastLog(lvl, `[RESPUESTA CMD]: ${rawData}`);
      }
    }
  });

  mqttClient.on('error', (err) => {
    broadcastLog('error', 'Error en MQTT', err.message);
    broadcastMqttStatus('error');
  });

  mqttClient.on('offline', () => {
    broadcastLog('warn', 'Broker MQTT fuera de línea.');
  });

  mqttClient.on('close', () => {
    broadcastMqttStatus('disconnected');
  });
}

function disconnectMQTT() {
  if (mqttClient) {
    mqttClient.end(true, () => {
      broadcastLog('warn', 'Conexión MQTT cerrada voluntariamente.');
      broadcastMqttStatus('disconnected');
      mqttClient = null;
    });
  } else {
    broadcastMqttStatus('disconnected');
  }
}

wss.on('connection', (ws) => {
  const currentStatus = mqttClient && mqttClient.connected ? 'connected' : 'disconnected';
  ws.send(JSON.stringify({ type: 'mqtt_status_change', status: currentStatus }));

  ws.on('message', (message) => {
    try {
      const parsed = JSON.parse(message);

      if (parsed.type === 'mqtt_connect') {
        connectMQTT();
      } else if (parsed.type === 'mqtt_disconnect') {
        disconnectMQTT();
      } else if (parsed.type === 'publish_motion') {
        if (!mqttClient || !mqttClient.connected) {
          broadcastLog('error', 'No se puede enviar comando: MQTT desconectado.');
          return;
        }

        // Publica en so101/cmd según el ACL
        const payloadStr = JSON.stringify(parsed.payload);
        mqttClient.publish(TOPIC_CMD, payloadStr, { qos: 1 }, (err) => {
          if (err) {
            broadcastLog('error', `Fallo al publicar en ${TOPIC_CMD}`, err.message);
          } else {
            broadcastLog('info', `Comando enviado a ${TOPIC_CMD}`);
          }
        });
      }
    } catch (err) {
      broadcastLog('error', 'Error en paquete recibido de WebSocket', err.message);
    }
  });
});

server.listen(PORT, () => {
  console.log(`[HTTP] Servidor listo en http://localhost:${PORT}`);
});