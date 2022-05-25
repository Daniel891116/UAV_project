# UAV-project

## Install ArduPilot SITL
Follow the instructions in this website:
https://ardupilot.org/dev/docs/building-setup-linux.html
## Install QGroundControl
Follow the instructions in this website:
https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/installing_qgc.md
## How to run

In 1st terminal
- run ArduPilot SITL
```bash
$ cd ~/ardupilot/ArduCopter/
$ sim_vehicle.py -v ArduCopter
```
In 2nd terminal
- run QGroundControl
```bash
$ ./QGroundControl.AppImage
```