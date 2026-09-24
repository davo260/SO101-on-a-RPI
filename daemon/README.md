# so101d — Demonio líder→seguidor

Capa de tiempo real del proyecto: un proceso con varios hilos que lee el brazo
líder, comanda el seguidor a 100 Hz y publica el estado en memoria compartida
POSIX para cualquier cliente (CLI, puente MQTT/sockets, ROS 2).

## Arquitectura

```
                         so101d (un proceso)
  /dev/so101_leader ──┐  ┌──────────────────────────────────────┐
                      ├─►│ hilo de control (100 Hz, SCHED_FIFO) │── seqlock ──► /dev/shm/so101
  /dev/so101_follower◄┘  │  único dueño de los dos buses        │               (estado)
                         ├──────────────────────────────────────┤
                         │ hilo supervisor (paso 2)             │◄── buzón + semáforos ── clientes
                         │  seguridad, comandos, watchdog       │    /so101_cmd_lock
                         ├──────────────────────────────────────┤    /so101_cmd_ready
                         │ hilo principal: señales, apagado     │
                         └──────────────────────────────────────┘
```

- **Estado (control → todos):** snapshot `so101_sample_t` protegido con un
  *seqlock*: el hilo de control nunca se bloquea; los lectores reintentan si
  leyeron durante una escritura.
- **Modo (supervisor → control):** `mode_req` atómico, leído una vez por ciclo.
- **Comandos (clientes → supervisor):** buzón en la misma shm, protegido por
  semáforos POSIX con nombre (paso 2).
- El contrato completo está en `include/so101_shm.h`.

### Ciclo de control
1. `clock_nanosleep(TIMER_ABSTIME)` sobre una rejilla fija.
2. SYNC_READ posiciones del líder.
3. SYNC_READ pos/vel/carga/voltaje/temp del seguidor (dir. 56..63).
4. Aplica el modo pedido: al activar torque primero escribe meta = posición
   actual (sin saltos); nunca activa torque si no pudo leer el seguidor.
5. Meta: mapeo líder→seguidor con la calibración de LeRobot
   (−100..100 en articulaciones, 0..100 en la pinza), limitada a
   `max_step` ticks por ciclo. SYNC_WRITE Goal_Position.
6. Publica en shm y cuenta overruns.

Modos: `IDLE` (torque off), `TELEOP`, `HOLD` (mantiene la pose de entrada),
`ESTOP` (torque off, enclavado).

## Compilar y probar
    make && make test

## Uso en la Pi
Cerrar LeRobot antes (el driver abre los puertos en exclusiva).

    CAL=~/.cache/huggingface/lerobot/calibration
    ls $CAL/teleoperators/so101_leader/ $CAL/robots/so101_follower/

    # modo IDLE: solo lee y publica (el seguidor queda sin torque)
    ./build/so101d -L $CAL/teleoperators/so101_leader/<id>.json \
                   -F $CAL/robots/so101_follower/<id>.json

    # en otra terminal
    ./build/so101ctl watch

    # teleoperación con tiempo real, hilo de control fijado al núcleo 3
    sudo ./build/so101d -r -C 3 -m teleop -L ... -F ...

Si `fts_tool raw` reportó eco en el adaptador, añadir `-e`.

## Sin hardware
    python3 ../driver/sim/fake_bus.py --wiggle   # líder simulado, imprime /dev/pts/A
    python3 ../driver/sim/fake_bus.py            # seguidor simulado, /dev/pts/B
    ./build/so101d -l /dev/pts/A -f /dev/pts/B -m teleop \
        -L tests/fixtures/so101_leader.json -F tests/fixtures/so101_follower_sim.json
    ./build/so101ctl watch
