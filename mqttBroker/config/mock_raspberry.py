import json
import random
import time
import paho.mqtt.client as mqtt

# Configuración del Broker y Credenciales
BROKER_HOST = "localhost"
BROKER_PORT = 1883
MQTT_USER = "dashboard"
MQTT_PASS = "Dash044#"

TOPIC_CONTROL = "robot/arm/control"
TOPIC_TELEMETRY = "robot/arm/telemetry"

# Estado simulado inicial del brazo robótico
current_joints = {
    "shoulder_pan": 90.0,
    "shoulder_lift": 45.0,
    "elbow_flex": 60.0,
    "wrist_flex": 45.0,
    "wrist_roll": 90.0,
    "gripper": 50.0,
}


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[Raspberry Pi Sim] Conectado exitosamente al broker Mosquitto")
        # Suscribirse a las órdenes enviadas por el Dashboard
        client.subscribe(TOPIC_CONTROL)
        print(f"[Raspberry Pi Sim] Escuchando comandos en: {TOPIC_CONTROL}")
    else:
        print(f"[Raspberry Pi Sim] Fallo al conectar. Código de retorno: {rc}")


def on_message(client, userdata, msg):
    global current_joints
    try:
        payload = json.loads(msg.payload.decode("utf-8"))
        print(
            f"\n[Raspberry Pi Sim] <-- Comando recibido en {msg.topic}: {payload.get('command')}"
        )

        if payload.get("command") == "MOVE_JOINTS":
            target_joints = payload.get("joints", {})
            print("[Raspberry Pi Sim] Moviendo servos a nuevas posiciones:")
            for joint, angle in target_joints.items():
                current_joints[joint] = angle
                print(f"   • {joint}: {angle}°")

        elif payload.get("command") == "EMERGENCY_STOP":
            print("[Raspberry Pi Sim] ¡¡PARADA DE EMERGENCIA RECIBIDA!!")

    except Exception as e:
        print(f"[Raspberry Pi Sim] Error al procesar mensaje: {e}")


def generate_telemetry():
    """Genera lecturas eléctricas y térmicas realistas para cada sección."""
    telemetry = {}
    for joint in current_joints.keys():
        # Valores simulados con pequeñas fluctuaciones naturales
        telemetry[joint] = {
            "temp": round(35.0 + random.uniform(0.5, 4.0), 1),  # ~35 a 39 °C
            "volt": round(11.8 + random.uniform(0.1, 0.4), 1),  # ~11.9 a 12.2 V
            "curr": round(0.20 + random.uniform(0.05, 0.45), 2),  # ~0.25 a 0.65 A
        }
    return telemetry


def main():
    client = mqtt.Client(client_id="mock_raspberry_pi_arm")
    client.username_pw_set(MQTT_USER, MQTT_PASS)

    client.on_connect = on_connect
    client.on_message = on_message

    print("[Raspberry Pi Sim] Conectando a Mosquitto...")
    client.connect(BROKER_HOST, BROKER_PORT, keepalive=60)
    client.loop_start()

    try:
        while True:
            # Publicar telemetría cada 1 segundo hacia el dashboard
            data = generate_telemetry()
            client.publish(TOPIC_TELEMETRY, json.dumps(data))
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[Raspberry Pi Sim] Deteniendo simulación...")
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
