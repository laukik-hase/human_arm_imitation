Build package arm_imitation
Copy scenes/libSimExtROSInterface.so and paste it in the CoppeliaSim parent folder
Start roscore
Start CoppeliaSim and load scenes/arm.ttt
Start simulation
rosrun arm_imitation publisher
enter 4 space separated angles in degree
Verify the movement of arm in simulation

