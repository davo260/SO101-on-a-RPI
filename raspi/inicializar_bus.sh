#!/bin/bash

set -Eeuo pipefail

# ============================================================
# CONFIGURACION
# ============================================================

BUS_MUX="./bus_mux.py"
CONFIG="./so101_bridge.ini"
SUMMARY="./resumen.txt"

LEADER_BUS="/dev/pts/4"
FOLLOWER_BUS="/dev/pts/5"

PID_LEADER=""
PID_FOLLOWER=""

# ============================================================
# VALIDACIONES
# ============================================================

if [[ "$EUID" -eq 0 ]]; then
    echo "[ERROR] Ejecuta este script sin sudo."
    exit 1
fi

[[ -f "$BUS_MUX" ]] || {
    echo "[ERROR] No existe $BUS_MUX"
    exit 1
}

[[ -f "$CONFIG" ]] || {
    echo "[ERROR] No existe $CONFIG"
    exit 1
}

[[ -e "$LEADER_BUS" ]] || {
    echo "[ERROR] No existe $LEADER_BUS"
    exit 1
}

[[ -e "$FOLLOWER_BUS" ]] || {
    echo "[ERROR] No existe $FOLLOWER_BUS"
    exit 1
}

# ============================================================
# FUNCIONES
# ============================================================

cleanup() {
    echo ""
    echo "Deteniendo BUS MUX..."

    [[ -n "$PID_LEADER" ]] && kill "$PID_LEADER" 2>/dev/null || true
    [[ -n "$PID_FOLLOWER" ]] && kill "$PID_FOLLOWER" 2>/dev/null || true
}

trap cleanup EXIT INT TERM

extract_port() {
    local logfile="$1"
    local keyword="$2"

    grep -i "$keyword" "$logfile" |
        grep -oE '/dev/pts/[0-9]+' |
        head -n 1 || true
}

update_ini() {
    local key="$1"
    local value="$2"

    if grep -qE "^[[:space:]]*${key}[[:space:]]*=" "$CONFIG"; then
        sed -i -E \
            "s|^[[:space:]]*${key}[[:space:]]*=.*|${key} = ${value}|" \
            "$CONFIG"
    else
        echo "${key} = ${value}" >> "$CONFIG"
    fi
}

get_ports() {
    local logfile="$1"
    local arm="$2"
    local elapsed=0

    BRIDGE_PORT=""
    DAEMON_PORT=""

    while (( elapsed < 20 )); do

        BRIDGE_PORT=$(extract_port "$logfile" "bridge")
        DAEMON_PORT=$(extract_port "$logfile" "daemon")

        if [[ -n "$BRIDGE_PORT" && -n "$DAEMON_PORT" ]]; then
            return 0
        fi

        if ! kill -0 "$CURRENT_PID" 2>/dev/null; then
            echo "[ERROR] bus_mux $arm terminó inesperadamente."
            cat "$logfile"
            return 1
        fi

        sleep 1
        ((elapsed+=1))
    done

    echo "[ERROR] Tiempo agotado esperando puertos de $arm."
    cat "$logfile"
    return 1
}

# ============================================================
# 1. INICIAR LEADER
# ============================================================

echo "=========================================="
echo "       INICIALIZACION SO101 BUS MUX"
echo "=========================================="

echo ""
echo "[1/5] Iniciando LEADER..."

python3 -u "$BUS_MUX" \
    --arm leader \
    --bus "$LEADER_BUS" \
    > bus_mux_leader.log 2>&1 &

PID_LEADER=$!
CURRENT_PID=$PID_LEADER

get_ports "bus_mux_leader.log" "leader"

LEADER_BRIDGE="$BRIDGE_PORT"
LEADER_DAEMON="$DAEMON_PORT"

echo "Leader bridge: $LEADER_BRIDGE"
echo "Leader daemon: $LEADER_DAEMON"

# ============================================================
# 2. INICIAR FOLLOWER
# ============================================================

echo ""
echo "[2/5] Iniciando FOLLOWER..."

python3 -u "$BUS_MUX" \
    --arm follower \
    --bus "$FOLLOWER_BUS" \
    > bus_mux_follower.log 2>&1 &

PID_FOLLOWER=$!
CURRENT_PID=$PID_FOLLOWER

get_ports "bus_mux_follower.log" "follower"

FOLLOWER_BRIDGE="$BRIDGE_PORT"
FOLLOWER_DAEMON="$DAEMON_PORT"

echo "Follower bridge: $FOLLOWER_BRIDGE"
echo "Follower daemon: $FOLLOWER_DAEMON"

# ============================================================
# 3. ACTUALIZAR ARCHIVO INI
# ============================================================

echo ""
echo "[3/5] Actualizando so101_bridge.ini..."

update_ini "LEADER_PORT" "$LEADER_BRIDGE"
update_ini "FOLLOWER_PORT" "$FOLLOWER_BRIDGE"

# ============================================================
# 4. CONFIGURAR DISPOSITIVOS DAEMON
# ============================================================

echo ""
echo "[4/5] Configurando dispositivos daemon..."

sudo ln -sf "$LEADER_DAEMON" /dev/so101_leader
sudo ln -sf "$FOLLOWER_DAEMON" /dev/so101_follower

sudo chmod 666 "$LEADER_DAEMON" "$FOLLOWER_DAEMON"

# ============================================================
# 5. GENERAR RESUMEN
# ============================================================

echo ""
echo "[5/5] Generando resumen.txt..."

cat > "$SUMMARY" <<EOF
==========================================
       RESUMEN CONFIGURACION SO101
==========================================

LEADER
Puerto fisico:       $LEADER_BUS
Puerto bridge:       $LEADER_BRIDGE
Puerto daemon:       $LEADER_DAEMON

FOLLOWER
Puerto fisico:       $FOLLOWER_BUS
Puerto bridge:       $FOLLOWER_BRIDGE
Puerto daemon:       $FOLLOWER_DAEMON

DISPOSITIVOS DAEMON
/dev/so101_leader   -> $LEADER_DAEMON
/dev/so101_follower -> $FOLLOWER_DAEMON

ARCHIVO INI
LEADER_PORT = $LEADER_BRIDGE
FOLLOWER_PORT = $FOLLOWER_BRIDGE

==========================================
EOF

cat "$SUMMARY"

echo ""
echo "=========================================="
echo "       INICIALIZACION COMPLETADA"
echo "=========================================="

echo "Leader PID: $PID_LEADER"
echo "Follower PID: $PID_FOLLOWER"

echo ""
echo "BUS MUX permanece activo."
echo "Presiona CTRL+C para detener ambos procesos."

wait "$PID_LEADER" &
wait "$PID_FOLLOWER" &
wait

