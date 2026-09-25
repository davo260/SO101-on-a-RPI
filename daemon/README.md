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
                         │ hilo supervisor (50 Hz)              │◄── buzón + semáforos ── clientes
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
  dos semáforos POSIX con nombre: `/so101_cmd_lock` (exclusión mutua entre
  clientes) y `/so101_cmd_ready` (despierta al supervisor). El cliente toma
  el lock, escribe, hace `sem_post(ready)`, espera `ack.id == id` y suelta el
  lock. Si un cliente muere con el lock tomado, el supervisor lo libera a los 2 s.
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

**Arranque suave:** en cada entrada a TELEOP/HOLD cada articulación avanza a
`-a` ticks/ciclo (6 ≈ 53 °/s) hasta alcanzar su objetivo (a menos de 50 ticks);
desde ahí esa articulación usa el límite de teleop `-s` (50). Es por
articulación para que un líder en movimiento no deje el brazo atrapado en la
rampa lenta.

### Supervisor
Único escritor de `mode_req` y `faults`. Espera en `sem_timedwait(ready, 20 ms)`:
un comando lo despierta al instante; si no, el timeout es su tick de 50 Hz.

| Falla | Condición | Acción |
|---|---|---|
| `STALL` | el lazo no publica en 200 ms | → ESTOP |
| `LEADER_COMM` | 10 lecturas fallidas seguidas del líder | TELEOP → HOLD |
| `FOLLOWER_COMM` | 10 lecturas fallidas seguidas del seguidor | → IDLE |
| `OVERTEMP` | algún servo ≥ `-T` (60 °C) durante 100 ms | → IDLE |
| `VOLTAGE` | alimentación fuera de 4.5–8.4 V durante 100 ms | → IDLE |
| `OVERRUN` | > 5 overruns en 1 s | solo aviso |
| `ESTOP` | comando `estop` | → ESTOP |

Las fallas quedan enclavadas: con fallas activas se rechaza `teleop`/`hold`
hasta un `reset` (si la causa persiste, la falla vuelve a aparecer).

### so101ctl
    so101ctl status | watch [hz]
    so101ctl teleop | hold | idle | estop | reset | ping

## Compilar y probar
    make && make test

## Uso en la Pi
Cerrar LeRobot antes (el driver abre los puertos en exclusiva).

    CAL=~/.cache/huggingface/lerobot/calibration
    ls $CAL/teleoperators/so_leader/ $CAL/robots/so_follower/

    # modo IDLE: solo lee y publica (el seguidor queda sin torque)
    ./build/so101d -L $CAL/teleoperators/so_leader/so101_leader.json \
                   -F $CAL/robots/so_follower/so101_follower.json

    # en otra terminal
    ./build/so101ctl watch

    # tiempo real, hilo de control fijado al núcleo 3; los modos por comando
    sudo ./build/so101d -r -C 3 -L ... -F ...
    ./build/so101ctl teleop      # arranque suave y luego teleoperación
    ./build/so101ctl hold
    ./build/so101ctl estop       # y luego: reset

Si `fts_tool raw` reportó eco en el adaptador, añadir `-e`.

## Servicio systemd
    cd daemon && sudo ./systemd/install.sh     # compila, instala, crea usuario so101
    sudo systemctl start so101d                # y 'enable' para arrancar al encender
    journalctl -u so101d -f
    so101ctl watch                             # tu usuario queda en el grupo so101

- `Type=notify`: el demonio avisa `READY=1` cuando sus hilos están arriba y
  publica su estado (`systemctl status so101d` muestra modo, ciclo y fallas).
- **Watchdog** (`WatchdogSec=2`): el hilo principal envía `WATCHDOG=1` solo si
  el lazo de control **y** el supervisor avanzaron desde la última revisión.
  Si alguno se cuelga, systemd mata el proceso y lo reinicia
  (`Restart=on-failure`). Al reiniciar, el arranque seguro deja el seguidor sin
  torque.
- Usuario sin privilegios `so101` (grupo `dialout` para los puertos) con
  `CAP_SYS_NICE` + `CAP_IPC_LOCK` para SCHED_FIFO y `mlockall` sin root.
  Sistema de archivos de solo lectura (`ProtectSystem=strict`), sin acceso a
  `/home`, solo sockets UNIX.
- Memoria compartida y semáforos con permisos `0660` y grupo `so101` (`-g so101`):
  solo los miembros del grupo pueden leer el estado o mandar comandos.
- `ExecStartPre` espera hasta 30 s a que existan `/dev/so101_leader` y
  `/dev/so101_follower`.
- Configuración: `/etc/default/so101d`. Calibración copiada a `/etc/so101/`.
- **Reconexión:** si un USB se desconecta, el hilo de control reabre el puerto
  cada 0.5 s; al volver, el supervisor lo informa y basta `so101ctl reset`.

Demostración del watchdog: `sudo kill -STOP $(pidof so101d)` congela el
proceso; a los 2 s `journalctl` muestra el timeout y systemd lo reinicia.

## Registro y gráfica de la teleoperación
`so101_log` es un cliente normal (sin tiempo real) de la memoria compartida:
lee el seqlock a 500 Hz y escribe cada ciclo nuevo una sola vez en un CSV; al
final informa cuántos ciclos no alcanzó a leer. No afecta al lazo.

    # en la Pi, con so101d corriendo
    so101_log -o teleop.csv -d 30          # 30 s (sin -d: hasta Ctrl+C)
    so101ctl teleop                        # y mover el líder

    # en el Mac
    scp samu@<ip-pi>:teleop.csv .
    python3 daemon/scripts/plot_teleop.py teleop.csv -o teleop.png

`plot_teleop.py` grafica líder y seguidor por articulación (unidades LeRobot)
y estima el **retraso** por correlación cruzada en el tramo más largo de
TELEOP después del arranque suave. En simulación: 10 ms (un ciclo) con
`-s 50` y ~440 ms con `-s 5` (el límite de velocidad domina).

## Sin hardware
    python3 ../driver/sim/fake_bus.py --wiggle   # líder simulado, imprime /dev/pts/A
    python3 ../driver/sim/fake_bus.py            # seguidor simulado, /dev/pts/B
    ./build/so101d -l /dev/pts/A -f /dev/pts/B -m teleop \
        -L tests/fixtures/so101_leader.json -F tests/fixtures/so101_follower_sim.json
    ./build/so101ctl watch
