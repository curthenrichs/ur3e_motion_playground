# ur3e_official_driver_relaxed_ik_demo
Demonstration of the official UR ROS robot driver with Relaxed-IK control

This is a test package created to prove out the new Universal Robots ROS driver.

# Package Requirements
- [fmauch/universal_robot](https://github.com/fmauch/universal_robot)
- [UniversalRobots/Universal_Robots_ROS_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)
- [uwgraphics/relaxed_ik](https://github.com/uwgraphics/relaxed_ik) or [Wisc-HCI/lively_ik](https://github.com/Wisc-HCI/lively_ik)
- [Wisc-HCI/robotiq_85_gripper](https://github.com/Wisc-HCI/robotiq_85_gripper)

## Getting Started

Follow UR Driver install / build instructions. For the UR3e in the lab, you do not need to install the URCaps as I have already done so. If using a different robot, then this would need to be done.

Notes for the Robotiq 85 Gripper. First, the UR polyscope cannot have both the Robotiq URCap and the remote serial URCap installed at the same time. I have uninstalled the Robotiq URCap. To control the gripper use a ROS driver instead. Second, the specific driver that works over the remote serial connection is provided in the Wisc-HCI fork of the Robotiq driver. I needed to increase the delay in several spots, reduce polling frequency, and increase timeout due to the longer latency in communication.

The UR driver affords calibration derived from the UR robots internal configuration. I have already created the calibration file in the subdirectory `calibration`.

Next run this package in the following order.

0. Launch `julia.launch`
  - This should be launched much earlier for the JIT compiler
1. Launch `arm.launch`
  - Enter the IP address of the robot as `robot_ip:=<IP>`
2. Start external communication URCap in polyscope on teach pendant
  - Robot can be set to local mode
  - In installation-URCaps, set the external communication URCap IP address to that of the ROS machine running the demo code
  - The program is only composed of the external communication command which can be found in the URCap section of the program primitives side-panel
3. Launch `gripper.launch`
  - The light on the Robotiq gripper should be solid blue
4. Launch `test.launch`

These steps will result in an Rviz window with an interactive marker to appear. Use the marker to define pose goals for the robot. A simple, two-button GUI will also be generated to open or close the gripper.

## Warning

If developing against the speed scaled position controller (as I did) then make sure to note what speed you were developing against. If developing on low speed (such as 10%) for robot control, then when set to full speed (100%) it may pose a safety or control stability risk.

I developed the Relaxed-IK and Lively-IK interfaces with `absolute` positioning. The default is set to `relative` and needs to be changed via hard-coding. My demo would need to be modified to allow for relative control.

## UR Modern Driver Notes
The following is to capture what needed to be done to use the previous driver (UR Modern Driver) with the UR3e.

1. UR Teach pendant must be set to remote-control mode.
2. UR Modern Driver takes control of robot by sending scripts over the standard TCP interface. One can send their own scripts using the `URScript` topic.
  - Unless pull-request was approved recently, one will need to find the most up-to-date fork supporting the e-series robots.
3. Robotiq control requires the URCap and can only be controlled via sending over the control script.
