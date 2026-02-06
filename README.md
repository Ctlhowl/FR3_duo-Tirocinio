# FR3_duo-Tirocinio
Sviluppo e configurazione di un ambiente di simulazione basato su ROS 2 e CoppeliaSim per sistemi di robot manipolatori collaborativi, finalizzato alla definizione e valutazione di compiti di manipolazione cooperativa bimanuale. 

## Primo Avvio Container
1. Al primo avvio in assoluto dobbiamo aggiungere il messaggio `sensor_msgs/msg/JointState` nel file `meta/interface.txt` e successivamente ricompilare il plugin `ROS2Interface` a CoppeliaSim per permettere la corretta comunicazione con ROS2.

```bash
source /opt/ros/humble/setup.bash
export COPPELIASIM_ROOT_DIR="/opt/coppelia"
cd /opt/coppelia/programming/ros2_packages/sim_ros2_interface/meta
echo "sensor_msgs/msg/JointState" >> interfaces.txt
cd /opt/coppelia/programming/ros2_packages
colcon build --symlink-install --packages-select sim_ros2_interface --cmake-args -DCOPPELIASIM_ROOT_DIR=$COPPELIASIM_ROOT_DIR
```
- In caso di errore: `Could not find a package configuration file provided by "pendulum_msgs"`. Aprire il file `CMakeLists.txt` e commentare tutte le righe relative al messaggio `pendulum_msgs`.

2. Clonare le dipendenze della repo franka_ros2 e Compliare l'intero progetto
```bash
cd /ros2_ws
sudo apt update && apt upgrade
vcs import /ros2_ws/src/franka_ros2 < /ros2_ws/src/franka_ros2/franka.repos --recursive --skip-existing
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

3. Estrarre il file `franka_descriptio.zip` in `src/franka_ros2`

### Avviare Rqt Graph
```bash
ros2 run rqt_graph rqt_graph
```

### Avviare Coppeliasim
```bash
source /opt/ros/humble/setup.bash
cd /opt/coppelia
./coppeliaSim.sh
```

In caso di errore `X11 connection` eseguire:
```bash
xhost +local:root
```

### Avviare Moveit Single Arm
```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit.launch.py load_gripper:=true robot_ip:=none use_sim:=true 
```

### Avviare Moveit Dual Arm
```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch franka_fr3_moveit_config moveit_duo.launch.py load_gripper:=true left_robot_ip:=none right_robot_ip:=none use_sim:=true 
```

### Avviare la scena Moveit Dual Arm
Liste dei task disponibili:
- dual_hand_changing
- start_pose

```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch fr3_motion_control fr3_motion_control.launch.py load_gripper:=true left_robot_ip:=none right_robot_ip:=none use_sim:=true task:=dual_hand_changing
```