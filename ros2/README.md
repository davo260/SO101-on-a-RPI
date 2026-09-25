# ROS 2: gemelo digital del SO-101 en RViz

El brazo real se refleja en un modelo 3D en RViz: el seguidor sólido y el líder
al lado, semitransparente. El puente es otro cliente (sin tiempo real) de la
memoria compartida de `so101d`, igual que `so101ctl` o `so101_log`.

```
Raspberry Pi                                        Mac (VM Ubuntu 24.04 + ROS 2 Jazzy)
so101d ─► /dev/shm/so101 ─► so101_bridge (Docker) ══ Zenoh/TCP 7447 ══► robot_state_publisher ─► RViz
                              /joint_states              ║   (URDF so101_description)
                              /so101/leader/joint_states ║
                              /so101/status   ◄── /so101/cmd (teleop, hold, estop...)
```

## Paquetes
| Paquete | Dónde corre | Qué hace |
|---|---|---|
| `so101_bridge` | Pi (Docker) | shm → `sensor_msgs/JointState` a 50 Hz (rad, rad/s, carga %), estado JSON a 1 Hz, `/so101/cmd` → comandos del supervisor |
| `so101_description` | Mac (VM) | URDF de TheRobotStudio (calibración "new": cero = mitad del rango) + launch y config de RViz |

Conversión a radianes (misma convención que el modo grados de LeRobot y que el
URDF "new calib"): `ángulo = (raw − (min+max)/2) · 2π / 4095`, con la
calibración de cada brazo. `config/bridge.yaml` tiene un signo y un offset por
articulación para ajustar el modelo una sola vez.

## 1. Pi: puente en Docker (+ router Zenoh)
Raspberry Pi OS no tiene paquetes de ROS 2; el contenedor `ros:jazzy` (arm64) sí.
Requisitos: Pi OS de 64 bits (`uname -m` → `aarch64`), `so101d` instalado como
servicio (`daemon/systemd/install.sh` deja la calibración en `/etc/so101/`).

    curl -fsSL https://get.docker.com | sudo sh     # una vez
    sudo usermod -aG docker $USER                   # y volver a entrar por SSH
    cd ~/SO101-on-a-RPI && ./ros2/docker/run_bridge.sh
    docker logs -f so101_bridge                     # "publishing /joint_states at 50 Hz"

El contenedor usa `--net host` (router Zenoh en `tcp/7447` accesible desde la
red) e `--ipc host` (ve `/dev/shm/so101` y los semáforos). Se reinicia solo con
la Pi. Para DDS clásico en una LAN con multicast: `RMW=rmw_fastrtps_cpp ./ros2/docker/run_bridge.sh`.

## 2. Mac: VM Ubuntu 24.04 (ARM64) en Parallels
Transporte: **Zenoh** (`rmw_zenoh_cpp`, oficial en Jazzy). El contenedor de la
Pi corre un router Zenoh en `tcp/7447`; la VM corre su propio router que se
conecta al de la Pi con una sola conexión TCP saliente. Funciona con la red
**Shared** (NAT) de Parallels y en redes que bloquean multicast (Wi-Fi del
campus); DDS clásico necesitaría Bridged + multicast.

1. VM Ubuntu **24.04 arm64 desktop** (`ubuntu-24.04.x-desktop-arm64.iso` de
   cdimage.ubuntu.com), red **Shared Network**. Comprobar: `uname -m` → aarch64,
   `nc -zv 10.195.22.101 22` → succeeded.
2. ROS 2 Jazzy (guía oficial "Ubuntu (deb packages)") y además:

       sudo apt install ros-jazzy-desktop ros-jazzy-rmw-zenoh-cpp ros-jazzy-joint-state-publisher-gui python3-colcon-common-extensions git
       echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
       echo 'export RMW_IMPLEMENTATION=rmw_zenoh_cpp' >> ~/.bashrc

3. Compilar el modelo:

       git clone https://github.com/davo260/SO101-on-a-RPI.git ~/SO101-on-a-RPI
       mkdir -p ~/ros2_ws/src && ln -s ~/SO101-on-a-RPI/ros2/src/so101_description ~/ros2_ws/src/
       ~/SO101-on-a-RPI/ros2/src/so101_description/fetch_model.sh   # URDF + mallas (Apache-2.0)
       cd ~/ros2_ws && colcon build --symlink-install
       echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc

4. Probar sin la Pi (sliders; con Zenoh hace falta un router local):
   terminal 1 `ros2 run rmw_zenoh_cpp rmw_zenohd`, terminal 2
   `ros2 launch so101_description display.launch.py gui:=true leader:=false`
5. Con la Pi:

       # terminal 1: router local enlazado al de la Pi
       ~/SO101-on-a-RPI/ros2/scripts/vm_zenoh_router.sh 10.195.22.101
       # terminal 2
       ros2 topic hz /joint_states          # ≈ 50 Hz
       ros2 launch so101_description display.launch.py
       # comandos desde la VM
       ros2 topic pub --once /so101/cmd std_msgs/msg/String "data: teleop"

Si no llegan tópicos: `nc -zv <ip-pi> 7447` desde la VM (el router de la Pi
debe aceptar), y que ambos lados usen `RMW_IMPLEMENTATION=rmw_zenoh_cpp`.
Para la presentación: cable Ethernet directo Pi ↔ Mac y la IP de la Pi en esa
interfaz como argumento de `vm_zenoh_router.sh`.

Revisar que los relojes estén sincronizados (`timedatectl` en ambos): si
difieren, RViz puede quejarse de TF "extrapolation into the future".

## 3. Ajuste del modelo (una vez)
Con `display.launch.py` corriendo, poner el seguidor en la pose media de la
calibración (la misma de `lerobot-calibrate`): el modelo debe verse igual. Si
una articulación gira al revés, cambiar su signo en `config/bridge.yaml`; si
queda corrida, poner su offset en radianes. Reconstruir la imagen
(`run_bridge.sh`) para aplicar.

## Pruebas sin ROS
    python3 clients/python/test_layout.py                     # layout Python == C
    python3 -m pytest ros2/src/so101_bridge/test               # conversión a radianes
