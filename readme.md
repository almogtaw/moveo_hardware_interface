# Moveo BCN3D Hardware Interface

## Compatibility
ROS2 Humble

  

## Done

- [x] Position control (position commands with optional velocity)
- [x] Velocity control
- [x] Position and velocity state interfaces
- [x] Arduino Mega suitable firmware
- [x] Added Joints limits - travel, velocity, acceleration

## Todo

- [ ] Organize joiunts limits values in parameter file
- [ ] Add info tags to read from ros2 control urdf
- [ ] Add gripper (servo) to hardware interface.
- [ ] Add gripper (servo) to Arduino firmware.


## Joits limits
based on real robot, 

| Joint  | Min travel  | Max travel  | Max velocity  | Nominal velocity  | Max acceleration  | Steps per radian  |
|--------|-------------|-------------|---------------|-------------------|-------------------|-------------------|
| 1      | -25,000     | +3,000      | 40,000        | 40,000            | 40,000            | 7162              |
| 2      | -6,100      | +7,000      | 60,000        | 30,000            | 50,000            | 3756              |
| 3      | -1,500      | +1,500      | 1,000         | 700               | 10,000            | 697               |
| 4      | ---         | ---         | ---           | ---               | ---               | ---               |
| 5      | -4,200      | +3,500      | 4,000         | 3,000             | 5,000             | 2228              |


the following joints controlled by TB6600 driver: 1, 2, 3.

Joint 4 inactive

Joint 5 controlled by A4988