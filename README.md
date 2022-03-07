# ur3e_motion_playground
Uses new UR driver with Lively-TK. Also provides MoveIt as a comparison.

This is a test package created to prove out the new Universal Robots ROS driver's joint trajectory controller servo behavior. Also provides a general reference on how to use some motion algorithms.

## Installation

Install following python dependencies with pip:
- [lively-tk==0.9.25](https://pypi.org/project/lively-tk/)
- python-tk

Install the following ROS packages: 
- [fmauch/universal_robot](https://github.com/fmauch/universal_robot)
- [UniversalRobots/Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
- [Wisc-HCI/robotiq_85_gripper](https://github.com/Wisc-HCI/robotiq_85_gripper)

## Execution


## Development


### Warning
If developing against the speed scaled position controller (as I did) then make sure to note what speed you were developing against. If developing on low speed (such as 10%) for robot control, then when set to full speed (100%) it may pose a safety or control stability risk.

I developed the Lively-TK interfaces with `absolute` positioning.

### UR Modern Driver Notes
The following is to capture what needed to be done to use the previous driver (UR Modern Driver) with the UR3e.

1. UR Teach pendant must be set to remote-control mode.
2. UR Modern Driver takes control of robot by sending scripts over the standard TCP interface. One can send their own scripts using the `URScript` topic.
  - Unless pull-request was approved recently, one will need to find the most up-to-date fork supporting the e-series robots.
3. Robotiq control requires the URCap and can only be controlled via sending over the control script.
