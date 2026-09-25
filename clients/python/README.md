# so101.py — cliente Python de so101d

Lee el estado (seqlock) y manda comandos (buzón + semáforos) sin tocar C.

    pip install posix_ipc          # solo para command()
    python3 so101.py               # vista en vivo
    python3 so101.py teleop        # comando

```python
from so101 import So101
arm = So101()                      # /dev/shm/so101 (usuario en el grupo so101)
st = arm.read()                    # dict: mode, faults, leader{pos,norm}, follower{pos,goal,norm,vel,load,volt,temp}, timing_us, counters
st = arm.wait_new(st["cycle"])     # bloquea hasta el siguiente ciclo (100 Hz)
arm.command("hold")                # "OK" | "REFUSED" | "BAD_CMD"
```

Unidades: `pos`/`goal` en ticks (0..4095), `norm` en unidades LeRobot
(−100..100, pinza 0..100), `volt` en V, `temp` en °C, `load` en 0.1 % del torque
máximo, tiempos en µs. `test_layout.py` verifica que la disposición de bytes
coincide con `daemon/include/so101_shm.h`.
