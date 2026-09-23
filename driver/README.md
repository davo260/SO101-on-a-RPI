# so101-driver — Driver Feetech STS3215 en C (termios)

Capa baja del proyecto: acceso directo al bus half-duplex TTL de los servos
STS3215 desde espacio de usuario en Linux, sin LeRobot ni el SDK de Feetech.

## Estructura
- `include/feetech.h`, `src/feetech.c` — driver: armado/verificación de tramas,
  termios a 1 Mbps, `poll()` con timeouts, resincronización ante basura,
  detección de eco, PING / READ / WRITE / SYNC_READ / SYNC_WRITE.
- `tools/fts_tool.c` — diagnóstico (solo lectura: nunca activa torque).
- `tests/test_protocol.c` — pruebas unitarias del protocolo, sin hardware.
- `sim/fake_bus.py` — bus simulado en un pty (eco y corrupción opcionales).

## Compilar y probar
    sudo apt install -y build-essential
    make && make test

## Uso con hardware (cerrar LeRobot antes: el driver abre el puerto en exclusiva)
    ./build/fts_tool /dev/so101_follower raw 1     # ¿el adaptador hace eco?
    ./build/fts_tool /dev/so101_follower scan      # IDs presentes
    ./build/fts_tool /dev/so101_follower state     # pos/vel/carga/voltaje/temp
    ./build/fts_tool /dev/so101_follower bench 5000  # latencia del bus
Si `raw` reporta eco, añadir `-e` a los demás comandos.

## Sin hardware
    python3 sim/fake_bus.py            # imprime /dev/pts/N
    ./build/fts_tool /dev/pts/N bench 2000
