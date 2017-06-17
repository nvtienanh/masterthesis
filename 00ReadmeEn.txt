************************************************** *******************
Sample program of "humanoid robot"
2005 April 15 Shuji Kajita
************************************************** *******************
[How to use]
All of the files in this directory and copy it to the appropriate location, Matlab command
Run the following at the line. In parentheses is the corresponding chapter number.

Chapter 2 kinematics
ulink_example: Programming with recursive call (2.4.2)
fk_random: Sets the random joint angle display the two-legged walking robot (2.5.2)
ik_random: view the robot to set a random feet position and orientation (2.5.3)
ik_random2: Same as ik_random but, using the inverse kinematics by the numerical solution (2.5.4)

Chapter 3 ZMP and dynamics
    calcurate_zmp: 2-legged walking robot of ZMP (IZMP) and display the center of gravity projection point (3.4.1)

Chapter 6 dynamics simulation
rigidbody_rotate: the rotational motion of a rigid body in zero gravity space animation (6.1.2)
screw_motion: motion of a rigid body with a constant space velocity vector (6.2.2)
rigidbody_fly: translational and rotational motion of a rigid body in zero gravity space (6.3.3)
top_simulation: frame of motion animation (6.3.4)
robot_simulation: the robot by the unit vector method dynamics simulation (6.4.3)

[Operation check]
Matlab for Windows ver.6.5, ver.7.0

-------------------------------------------------- -----------------
? Remarks 0: promise of attention
This program is no warranty. For any damage caused by the result of the use of this program
We can not respond.

? Note 1: The file name of assigning rules (but with some exceptions)
Things can be run directly: file name is all lower case (for example: ulink_example.m)
Auxiliary function (argument is required): are used case in the file name (for example: PrintLinkName.m)

? Note 2: In the case of Linux version of Matlab
Besides Japanese in my environment (Vine Linux2.6 + Matlab ver.6.5) it can not be displayed was possible execution.

? Note 3: If the graphic display is strange
If the display of three-dimensional shape is strange, it might be improved by running the following at the command line.
set (0, 'DefaultFigureRenderer', 'zbuffer')