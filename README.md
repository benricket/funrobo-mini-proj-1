# Mini Project 1
Ben Ricket, Grant Rechtin

## Overview
This repository and project includes the implementation of forward position kinematics (FPK) using a given simulator for two different robot arms, the *Hiwonder 5DOF Arm* and the *Kinova 6DOF Arm*, along with forward velocity kinematics (FVK) and reduced rate motion control (RRMC) for the Hiwonder robot. This project also includes the implementation for connecting to, and manipulating, the Hiwonder using a controller. 

## Setup & Usage
For the initial setup you need to clone 3 repositories, this, as well as the two linked here:
[funrobo_hiwonder](https://github.com/OlinCollege-FunRobo/funrobo_hiwonder) 
[funrobo_kinematics](https://github.com/OlinCollege-FunRobo/funrobo_kinematics) 

The FPK code relies on helper functions and class templates defined in the `funrobo_kinematics` repository, and the FVK code for the physical Hiwonder arm relies on helper functions for connecting to the arm defined in the `funrobo_hiwonder` repository. The code in each of these repositories can be added as a pip package according to the instructions on the `funrobo_hiwonder` README. 

Once both of these have been cloned, follow the setup instruction through step 3 within the funrobo_hiwonder readme file. Once this has been complete, within this projects repository, while in the new conda environment created as per the instructions in the `funrobo_hiwonder` README, you can run the simulators with the code below:

Script to run the 5DOF arm visualizer (implements FPK and FVK). The X, Y, and Z axes are controlled via the arrow keys and W + S
5DOF Hiwonder Simulator
```terminal
python scripts/5dof_hiwonder.py
```
Script to run the 6DOF arm visualizer (implements FPK)
6DOF Kinova Simulator
```terminal
python scripts/6dof_kinova.py
```

For the connection to and implementation of 5DOF arm, the hiwonder first needs to be powered on and connected to the same network as your computer. Following this, you can connect through the terminal command:
```terminal
ssh pi@{specific robot handle}.local
```

The robot must have the same repositories and conda environment set up to run the code. 
Once this has been complete, within the conda environment made previously, you can run this command to control the robot using a controller plugged into the robot:
```terminal
python scripts/hiwonder_rrmc.py
```
The X, Y, and Z of the arm can be controlled via the two joysticks. 
