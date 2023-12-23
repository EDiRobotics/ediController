# EDI ENV

## Notice

In the following part, if you are executing on *(pc@lab)*, you need not to
run `source /home/pc/ediControler/catkin_ws/devel/setup.bash`

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

## How to record

1. **(New) Run `source /home/pc/ediControler/catkin_ws/devel/setup.bash`**

2. Run recording program `python data_collection/bag_record.py`.

```python
if __name__ == "__main__":
    start_record_service(None)
    time.sleep(5)
    end_record_service(None)
    convert_thread.join()
```

Currently it will record a 5 seconds episode and convert it to lmdb datasets.

### File Organization

The dataset is organized into the following directory structure for efficient access and management:

- `dataset/`
    - `bag/`
        - This directory contains the bag files. These are typically used for storing and transporting a collection of
          files.
    - `train_xxx_lmdb/`
        - This directory contains the LMDB (Lightning Memory-Mapped Database) files for training. LMDB is used for its
          high performance and efficiency in reading/writing large data sets.

## How to replay

Please check the python files for passing arguments.
### Replaying rosbag file (experimental)

1. **(New) Run `source /home/pc/ediControler/catkin_ws/devel/setup.bash`**

2. Run `python data_collection/bag_loader.py`.

### Replaying lmdb file (experimental)

1. **(New) Run `source /home/pc/ediControler/catkin_ws/devel/setup.bash`**

2. Run `python data_collection/lmdb_interface.py`.

## Run with Docker on *timetserver*

To run in a docker container, you need to run

```bash
CUDA="--gpus all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility -e NVIDIA_VISIBLE_DEVICES=all "
sudo docker run -it --rm --net=host --shm-size 32G \
-v <your directory>:/workspace/<docker directory> \
$CUDA edi/ros
```



