# Interfaz de so101d para la capa de red (MQTT / sockets / dashboard)

`so101d` corre en la Pi como servicio systemd. Publica el estado del brazo en
memoria compartida POSIX y recibe comandos por un buzón con semáforos. La capa
de red es **un cliente más**: lee y manda comandos con `clients/python/so101.py`,
sin tocar C y sin afectar el lazo de tiempo real.

```
so101d (C, 100 Hz) ──► /dev/shm/so101 ──► puente de red (Python) ──MQTT/sockets──► servidor / dashboard
                  ◄── buzón + semáforos ◄──        (comandos)       ◄──────────────
```

## Acceso en la Pi
    sudo adduser <usuario>                  # si no tiene cuenta
    sudo usermod -aG so101 <usuario>        # permiso de lectura/comandos (volver a entrar)
    sudo apt install python3-posix-ipc      # semáforos desde Python
    python3 ~/SO101-on-a-RPI/clients/python/so101.py        # vista en vivo

## API Python
```python
import sys; sys.path.insert(0, "/home/<usuario>/SO101-on-a-RPI/clients/python")
from so101 import So101
arm = So101()
st = arm.read()                     # snapshot consistente (dict), ver abajo
st = arm.wait_new(st["cycle"])      # espera el siguiente ciclo (100 Hz) o None
res = arm.command("teleop")         # "OK" | "REFUSED" | "BAD_CMD"
arm.alive()                         # False si so101d se detuvo
```
`read()` es barato (copia de 192 bytes); para un dashboard basta leer a
10–20 Hz con un temporizador. `command()` bloquea como máximo ~1 s.

## Estado (`read()`)
| Campo | Tipo / unidades | Notas |
|---|---|---|
| `cycle` | int | contador del lazo de control (100 por segundo) |
| `t` | s (CLOCK_MONOTONIC) | para calcular tasas, no es hora del día |
| `mode` | `IDLE` \| `TELEOP` \| `HOLD` \| `ESTOP` | modo aplicado por el lazo |
| `torque`, `ramping` | bool | torque del seguidor; arranque suave en curso |
| `faults` | lista de nombres | ver tabla de fallas; `fault_bits` = máscara |
| `joints` | 6 nombres | shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper |
| `leader.ok` / `follower.ok` | bool | lectura del bus exitosa en este ciclo |
| `leader.pos`, `follower.pos`, `follower.goal` | ticks 0..4095 | crudos del servo |
| `leader.norm`, `follower.norm` | −100..100 (pinza 0..100) | unidades LeRobot, comparables entre brazos |
| `follower.vel` | ticks/s | con signo |
| `follower.load` | 0.1 % del torque máx. | con signo |
| `follower.volt` | V | ~5.0 con la fuente actual |
| `follower.temp` | °C | falla OVERTEMP a 60 °C |
| `timing_us` | µs | `wake_lat`, `exec`, `bus_leader`, `bus_follower` y sus máximos |
| `counters` | int | `overruns`, `missed`, `leader_errs`, `follower_errs` |

## Comandos (`command(nombre)`)
| Comando | Efecto | Se rechaza (`REFUSED`) si |
|---|---|---|
| `teleop` | el seguidor copia al líder (con arranque suave) | hay cualquier falla activa |
| `hold` | el seguidor se queda en la pose actual | hay fallas (excepto LEADER_COMM) |
| `idle` | torque apagado | está en ESTOP (usar `reset`) |
| `estop` | torque apagado, enclavado | nunca |
| `reset` | borra fallas y pasa a IDLE | nunca (si la causa sigue, la falla reaparece) |
| `ping` | nada, solo responde | nunca |

El dashboard debe mostrar la respuesta: `REFUSED` significa "hay fallas: mira
`faults` y usa `reset`".

## Fallas
| Nombre | Causa | Acción automática |
|---|---|---|
| `LEADER_COMM` | USB/bus del líder sin respuesta | TELEOP → HOLD |
| `FOLLOWER_COMM` | USB/bus del seguidor sin respuesta | → IDLE |
| `OVERTEMP` | servo ≥ 60 °C | → IDLE |
| `VOLTAGE` | alimentación fuera de 4.5–8.4 V | → IDLE |
| `OVERRUN` | el lazo pierde plazos seguido | solo aviso |
| `STALL` | el lazo dejó de publicar | → ESTOP (systemd reinicia) |
| `ESTOP` | comando `estop` | → ESTOP |
| `TORQUE_REFUSED` | no se pudo leer el seguidor al activar torque | queda en el modo anterior |

## Propuesta MQTT (a decidir por la capa de red)
| Tópico | Dirección | Contenido | Tasa |
|---|---|---|---|
| `so101/state` | Pi → broker | JSON: `cycle, mode, torque, faults, follower.norm, leader.norm, volt, temp` | 10–20 Hz |
| `so101/event` | Pi → broker | JSON al cambiar `mode` o `faults` | por evento |
| `so101/cmd` | broker → Pi | `"teleop"`, `"hold"`, `"idle"`, `"estop"`, `"reset"` | por evento |
| `so101/cmd/result` | Pi → broker | `{"cmd": ..., "result": "OK" \| "REFUSED" \| ...}` | por evento |

Sugerencias: QoS 1 para `cmd` y `event`, QoS 0 para `state`; `estop` siempre
debe estar a un clic en el dashboard.

## Desarrollar sin los brazos
En cualquier Linux (la Pi o una VM), desde la raíz del repo:

    cd daemon && make && cd ..
    python3 driver/sim/fake_bus.py --wiggle     # líder simulado: imprime /dev/pts/A
    python3 driver/sim/fake_bus.py              # seguidor simulado: /dev/pts/B
    daemon/build/so101d -l /dev/pts/A -f /dev/pts/B \
        -L daemon/tests/fixtures/so101_leader.json -F daemon/tests/fixtures/so101_follower_sim.json
    python3 clients/python/so101.py             # el líder simulado se mueve solo

(Sin `-g`, la memoria compartida queda con permisos 0666: no hace falta el grupo.
En la Pi, si el servicio `so101d` está corriendo, no lances un segundo demonio:
usa el real, o `sudo systemctl stop so101d` antes de simular.)
