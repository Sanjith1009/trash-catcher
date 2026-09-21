### 1. Remote & Local Setup

1. **SSH Connection:** Connect to the remote environment using `my_cluster`.
```bash
ssh my_cluster

```


2. **VS Code Workspace:** Open the `src` folder in VS Code (no terminal navigation needed).
3. **Run Bridge Scripts:** Open two separate VS Code terminal windows and run:
```bash
py local_bridge.py

```


```bash
py webcam_bridge.py

```



---

### 2. Podman Container Setup

Launch a new Podman container:

```bash
podman run -it --rm \
  --net=host \
  -v ~/ros2_ws:/ros2_ws \
  -w /ros2_ws \
  docker.io/osrf/ros:humble-desktop bash

```

> **Note:** If starting a new session on the cluster, you may be in a different workspace and need to reinstall `pyserial`:
> ```bash
> apt-get -o APT::Sandbox::User=root update && apt-get -o APT::Sandbox::User=root install -y python3-serial
> 
> ```
> 
> 

---

### 3. ROS 2 Workspace Setup & Execution

#### Step 1: Rebuild Packages *(Only necessary after making code changes)*

```bash
colcon build --packages-select trash_catcher_pkg --symlink-install
colcon build --packages-select webcam_publisher
colcon build --packages-select vision_tracker 

```

#### Step 2: Source Workspace

Run this inside `ros2_ws` first:

```bash
source install/setup.bash

```

#### Step 3: Run Nodes

```bash
ros2 run trash_catcher_pkg task_node
ros2 run webcam_publisher webcam_publisher_node
ros2 run vision_tracker tracker_node 

```
