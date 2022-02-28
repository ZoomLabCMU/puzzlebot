# PuzzleBots: Physical Coupling of Robot Swarms
Sha Yi, Zeynep Temel, Katia Sycara

<img src="/img/cross_gap_demo.jpg" width="80%">

Link to our [video](https://www.youtube.com/watch?v=QP3eMZXLSw4). We present a guide to make and run our PuzzleBots.  

If you find our work useful, please consider citing our paper:  
```
@inproceedings{yi2021puzzlebots,
  title={PuzzleBots: Physical Coupling of Robot Swarms},
  author={Yi, Sha and Temel, Zeynep and Sycara, Katia},
  booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={8742--8748},
  year={2021},
  organization={IEEE}
}
```

**Software Requirement**
- python
- ROS
- Arduino

**Hardware Requirement**
- 3D Printer
- Soldering Iron
- Circuit Board Production (may use external manufacturer)
- Vicon (or other localization systems)

## Preparation
1. Assemble the robot based on instructions from the [assembly page](docs/assembly.md).
2. Burn bootloader of Arduino Pro Mini (8M, 3.3V) to the ATMega328P.
3. Configure ESP8266 WiFi module with baud rate to be 9600.

## Run the Demo
### Setup
1. Configure the system  
Change Line 83 and 105 in `code/puzzlebot_arduino/wifitcp_motor_only/wifitcp_motor_only.ino` to your own wifi and IP address.
2. Load program onto circuit board  
Open the program `code/puzzlebot_arduino/wifitcp_motor_only/wifitcp_motor_only.ino` in Arduino Studio (or other Software) and load to the circuit board built in [assembly page](docs/assembly.md).
3. Build ROS package  
Copy `code/puzzlebot_control/` into the ROS catkin source directory. Build the package.

### Run
Each robot is associated with an IP address. In our setting, robot IP address has the form `192.168.0.20[ROBOT_ID]`. The corresponding Vicon ROS topic thus has the form `vicon/p20[ROBOT_ID]/p20[ROBOT_ID]`. For example, robot #2 has an IP of `192.168.0.202`, and the corresponding vicon topic is `vicon/p202/p202`.  
This can be changed in `code/puzzlebot_control/src/puzzlebot_control/hardware_wrap.py` Line 33. To turn off the vicon callback, change Line 30 in the `hardware_wrap.py` file.

Before launching the puzzlebot node, make sure the Vicon (or other localization system) is running and publishing the corret ROS topics. Then do
```
rosrun puzzlebot_control run_hardware.py [NUMBER_OF_ROBOTS]
```
For example, with three robots, run `rosrun puzzlebot_control run_hardware.py 3`.

Then, switch on the robots by sliding the switch on the top. You are expected to see the ESP8266 LED flashes then the console will show robots are connected in sequence. The program will start once all robots are connected.

### Testing Functions
A testing function `keyboard_input_lr` is provided in `robot.py`. It can be called in `hardware_wrap.py` to send velocity command during runtime to the robots to help debug hardware issues.
