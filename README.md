# EDI ENV

### Gym Env

##### Usage

1. Set hosts with on both **master** *(pc@lab)* and **slaves**:

Add to your hosts file with `sudo vim /etc/hosts`:
```bash
192.168.1.240   lab
192.168.1.250   timetserver
192.168.1.38    radiance_wired
192.168.1.147   radiance_wireless
```
2. Run `start_publishing.sh` on master (pc@lab).

3. Add `edi_gym` directory to your directory.
4. Set environment variable on your computer.
```bash
export ROS_HOSTNAME=radiance_wired # Replace this hostname
export ROS_MASTER_URI=http://192.168.1.240:11311 # Do not Replace this one
```
5. Run `python edi_env_example.py`
