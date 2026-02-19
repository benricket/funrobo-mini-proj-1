# Mini Project 1
## Overview
This repository and project include the implementation of a simulator for two different robot arms, the *Hiwonder 5DOF Arm* and the *Kinova 6DOF Arm*, along with the implementation for connecting to, and manipulating, the *Hiwonder 5DOF Arm* using a controller. 

## Setup
For the initial setup you need to clone 3 repositories, this, as well as the two linked here:
[Funrobo_hiwonder](https://github.com/OlinCollege-FunRobo/funrobo_hiwonder) 
[Funrobo_kinematics](https://github.com/OlinCollege-FunRobo/funrobo_kinematics) 

Once both of these have been cloned, follow the setup instruction through step 3 within the funrobo_hiwonder readme file. Once this has been complete, within this projects repository, while in the new conda environment, you can run the simulators with the code below:

5DOF Hiwonder Simulator
```terminal
python scripts/5dof_hiwonder.py
```
6DOF Kinova Simulator
```terminal
python scripts/6dof_kinova.py
```

For the connection to and implementation of 5DOF arm, the hiwonder first needs to be connected to the same wifi as your computer. Following this, you can connect through the terminal command:
```terminal
ssh pi@{specific robot handle}.local
```

Once this has been complete, within the conda environment made previously you can run this command to control the robot using a controller plugged into the robot:
```terminal
python scripts/hiwonder_rrmc.py
```
