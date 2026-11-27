#!/usr/bin/env python3
"""
bus_mux.py - Multiplexor de bus serie: un bus (fake_bus.py), dos clientes.

  python3 bus_mux.py --arm leader   --bus /dev/pts/3
  python3 bus_mux.py --arm follower --bus /dev/pts/4 [--stats]

Crea dos ptys con enlaces estables:
  /tmp/so101_<arm>_daemon   -> para so101d
  /tmp/so101_<arm>_bridge   -> para mqtt_bridge.py

Cada petición se reenvía al bus de una en una; la respuesta vuelve solo al cliente
que preguntó. El demonio tiene prioridad sobre el puente.
Por defecto solo imprime los puertos al arrancar y un resumen al salir.
"""
import argparse
import os
import select
import sys
import time
import tty
from collections import deque

BROADCAST = 0xFE


def open_pty():
    master, slave = os.openpty()
    tty.setraw(slave)
    tty.setraw(master)
    os.set_blocking(master, False)
    return master, slave


def take_frames(buf):
    """Extrae tramas completas FF FF ID LEN ... (sin validar checksum)."""
    out = []
    while True:
        i = buf.find(b"\xff\xff")
        if i < 0:
            if buf and buf[-1] == 0xFF:
                del buf[:-1]
            else:
                buf.clear()
            break
        if i:
            del buf[:i]
        if len(buf) < 4:
            break
        total = buf[3] + 4
        if len(buf) < total:
            break
        out.append(bytes(buf[:total]))
        del buf[:total]
    return out


def expected_responses(frame):
    """Cuántas tramas de estado produce esta instrucción."""
    sid, inst, params = frame[2], frame[4], frame[5:-1]
    if inst == 0x82:                       # SYNC_READ: params = [addr, len, ids...]
        return max(0, len(params) - 2)
    if sid == BROADCAST:
        return 0
    if inst in (0x01, 0x02, 0x03, 0x04, 0x06):   # PING, READ, WRITE, REG_WRITE, RESET
        return 1
    return 0                               # SYNC_WRITE (0x83), ACTION, ...


class Client:
    def __init__(self, name, link):
        self.name = name
        self.master, self.slave = open_pty()   # el slave se mantiene abierto aquí
        self.buf = bytearray()
        self.queue = deque()                   # (t_encolado, trama)
        self.link = link
        if os.path.lexists(link):
            os.remove(link)
        os.symlink(os.ttyname(self.slave), link)
        self.reqs = 0
        self.max_wait = 0.0
        self.timeouts = 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", required=True, choices=["leader", "follower"])
    ap.add_argument("--bus", required=True, help="puerto de fake_bus.py (ej. /dev/pts/3)")
    ap.add_argument("--timeout", type=float, default=30.0,
                    help="ms máximos esperando respuesta del bus (default 30)")
    ap.add_argument("--stats", action="store_true",
                    help="imprimir un resumen por segundo (apagado por defecto)")
    a = ap.parse_args()

    bus = os.open(a.bus, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(bus)
    os.set_blocking(bus, False)

    clients = {
        "daemon": Client("daemon", f"/tmp/so101_{a.arm}_daemon"),
        "bridge": Client("bridge", f"/tmp/so101_{a.arm}_bridge"),
    }
    order = ["daemon", "bridge"]           # prioridad
    by_master = {c.master: c for c in clients.values()}
    bus_buf = bytearray()

    print(f"[{a.arm}] bus: {a.bus}")
    for c in clients.values():
        print(f"[{a.arm}] puerto para {c.name}: {c.link} -> {os.ttyname(c.slave)}")
    print(f"[{a.arm}] en marcha. Ctrl+C para salir.", flush=True)

    current = None          # dict: client, expected, got, deadline, t_sent
    stray = 0
    lat_sum, lat_n, lat_max = 0.0, 0, 0.0
    last_print = time.time()

    try:
        while True:
            now = time.monotonic()
            wait = 0.05
            if current:
                wait = max(0.0, min(wait, current["deadline"] - now))
            r, _, _ = select.select([bus] + list(by_master), [], [], wait)
            now = time.monotonic()

            # --- peticiones de los clientes ---
            for fd in r:
                c = by_master.get(fd)
                if c is None:
                    continue
                try:
                    data = os.read(fd, 4096)
                except (BlockingIOError, OSError):
                    continue
                c.buf.extend(data)
                for fr in take_frames(c.buf):
                    c.queue.append((now, fr))

            # --- respuestas del bus ---
            if bus in r:
                try:
                    data = os.read(bus, 4096)
                except BlockingIOError:
                    data = b""
                except OSError:
                    print(f"[{a.arm}] el bus {a.bus} se cerró; saliendo.", flush=True)
                    break
                bus_buf.extend(data)
                for fr in take_frames(bus_buf):
                    if current is None:
                        stray += 1
                        continue
                    try:
                        os.write(current["client"].master, fr)
                    except (BlockingIOError, OSError):
                        pass
                    current["got"] += 1
                    if current["got"] >= current["expected"]:
                        lat = now - current["t_sent"]
                        lat_sum += lat
                        lat_n += 1
                        lat_max = max(lat_max, lat)
                        current = None

            # --- timeout de la petición en vuelo ---
            if current and now > current["deadline"]:
                current["client"].timeouts += 1
                current = None

            # --- despachar la siguiente petición (el demonio primero) ---
            while current is None:
                nxt = next((clients[n] for n in order if clients[n].queue), None)
                if nxt is None:
                    break
                t_enq, fr = nxt.queue.popleft()
                nxt.reqs += 1
                nxt.max_wait = max(nxt.max_wait, now - t_enq)
                os.write(bus, fr)
                exp = expected_responses(fr)
                if exp > 0:
                    current = {"client": nxt, "expected": exp, "got": 0,
                               "t_sent": now, "deadline": now + a.timeout / 1000.0}

            # --- resumen por segundo (solo con --stats) ---
            if a.stats and time.time() - last_print >= 1.0:
                d, b = clients["daemon"], clients["bridge"]
                avg = (lat_sum / lat_n * 1000) if lat_n else 0.0
                print(f"[{a.arm}] reqs daemon={d.reqs} bridge={b.reqs} | "
                      f"timeouts daemon={d.timeouts} bridge={b.timeouts} stray={stray} | "
                      f"latencia bus avg={avg:.2f}ms max={lat_max * 1000:.2f}ms | "
                      f"espera en cola max daemon={d.max_wait * 1000:.2f}ms "
                      f"bridge={b.max_wait * 1000:.2f}ms", flush=True)
                d.max_wait = b.max_wait = 0.0
                lat_sum, lat_n, lat_max = 0.0, 0.0, 0.0
                lat_n = 0
                last_print = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        d, b = clients["daemon"], clients["bridge"]
        print(f"\n[{a.arm}] resumen final: reqs daemon={d.reqs} bridge={b.reqs} | "
              f"timeouts daemon={d.timeouts} bridge={b.timeouts} | stray={stray}", flush=True)
        for c in clients.values():
            if os.path.islink(c.link):
                os.remove(c.link)


if __name__ == "__main__":
    main()
