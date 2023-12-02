# EDI ENV

### Gym Env

##### Usage

1. Set hosts with on both **master** *(pc@lab)* and **slaves**

Add to your hosts file with `sudo vim /etc/hosts`:
```bash
192.168.1.240   lab
192.168.1.250   timetserver
192.168.1.38    radiance_wired
192.168.1.147   radiance_wireless
```
2. Run `start_publishing.sh` on master (pc@lab).

3. Set environment variable on your computer.
```bash
export ROS_HOSTNAME=timetserver # Replace this hostname
export ROS_MASTER_URI=http://192.168.1.240:11311 # Do not Replace this one
```

4. Add `edi_gym` directory to your directory.

5. Run `python edi_env_example.py`

##### Run with docker on *timetserver*

To run in a docker container:
```bash
CUDA="--gpus all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility -e NVIDIA_VISIBLE_DEVICES=all "
sudo docker run -it --rm --net=host --shm-size 32G \
-v <your directory>:/workspace/<docker directory> \
$CUDA edi/ros
```

To run a test example:
```bash
cd ediControler
python edi_env_example.py
```

CUDA="--gpus all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility -e NVIDIA_VISIBLE_DEVICES=all "
sudo docker run -it --rm --net=host --shm-size 32G \
-v /home/radiance/ediControler:/workspace/ediControler \
$CUDA edi/ros ldconfig