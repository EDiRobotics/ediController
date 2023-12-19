# EDI ENV

## Install (New)

#### Compile ros rpc server call dependency.

```bash
cd catkin_ws
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## How to run

### Setups

This section includes ROS Configurations and starting several hardware interfaces.

1. Set hosts with on both **master** *(pc@lab)* and **slaves**

Add to your hosts file with `sudo vim /etc/hosts`:

```bash
192.168.1.240   lab
192.168.1.250   timetserver
192.168.1.38    radiance_wired
192.168.1.147   radiance_wireless
```

2. Run `start_publishing.sh` on master (pc@lab).

        It will firstly `source /home/pc/ediControler/catkin_ws/devel/setup.bash` and then start nodes below:
        ```
        Camera Nodes
        Status Node
        Real Environment Control Backend
        Sim Environment
        ```

3. Set environment variable on your computer.

```bash
export ROS_HOSTNAME=timetserver # Replace this hostname
export ROS_MASTER_URI=http://192.168.1.240:11311 # Do not Replace this one
```

### Policy Inference

4. Add `edi_gym` directory to your directory.

5. **(New) Run `source /home/pc/ediControler/catkin_ws/devel/setup.bash`**

6. Run a test example `python edi_env_example.py`.

### Human Demonstration

4. **(New) Run `source /home/pc/ediControler/catkin_ws/devel/setup.bash`**

5. Run demo arm `python hardware/arm_demo.py`.

## Run with Docker on *timetserver*

To run in a docker container, you need to run

```bash
CUDA="--gpus all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility -e NVIDIA_VISIBLE_DEVICES=all "
sudo docker run -it --rm --net=host --shm-size 32G \
-v <your directory>:/workspace/<docker directory> \
$CUDA edi/ros
```