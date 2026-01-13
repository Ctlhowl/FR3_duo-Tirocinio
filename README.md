# FR3_duo-Tirocinio
Sviluppo e configurazione di un ambiente di simulazione basato su ROS 2 e CoppeliaSim per sistemi di robot manipolatori collaborativi, finalizzato alla definizione e valutazione di compiti di manipolazione cooperativa bimanuale. 

## Primo Avvio Container
Al primo avvio in assoluto dobbiamo aggiungere il messaggio `sensor_msgs/msg/JointState` nel file `meta/interface.txt` e successivamente ricompilare il plugin `ROS2Interface` a CoppeliaSim per permettere la corretta comunicazione con ROS2.

```bash
source /opt/ros/humble/setup.bash
export COPPELIASIM_ROOT_DIR="/opt/coppelia"
cd /opt/coppelia/programming/ros2_packages/sim_ros2_interface/meta
echo "sensor_msgs/msg/JointState" >> interface.txt
colcon build --symlink-install --packages-select sim_ros2_interface --cmake-args -DCOPPELIASIM_ROOT_DIR=$COPPELIASIM_ROOT_DIR
```

In caso di errore: `Could not find a package configuration file provided by "pendulum_msgs"`. Aprire il file `CMakeLists.txt` e commentare tutte le righe relative al messaggio `pendulum_msgs`.

### Avviare Coppeliasim
```bash
source /opt/ros/humble/setup.bash
cd /opt/coppelia
./coppeliaSim.sh
```