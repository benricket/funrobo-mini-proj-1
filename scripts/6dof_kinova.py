from math import *
import numpy as np
from funrobo_kinematics.core.visualizer import Visualizer, RobotSim
import funrobo_kinematics.core.utils as ut
from funrobo_kinematics.core.arm_models import KinovaRobotTemplate

class KinovaRobot(KinovaRobotTemplate):
    def __init__(self):
        super().__init__()
    
    def dh_to_H(self,dh_table):
        H_list = []
        for i in range(dh_table.shape[0]):
            theta = dh_table[i,0]
            d = dh_table[i,1]
            a = dh_table[i,2]
            alpha = dh_table[i,3]
            H_list.append(np.array([
                [cos(theta),-sin(theta)*cos(alpha),sin(theta)*sin(alpha),a*cos(theta)],
                [sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta)],
                [0,sin(alpha),cos(alpha),d],
                [0,0,0,1]
            ]))

            if (i == 0):
                H_ee = H_list[0]
            else:
                # build on previous H
                H_ee = H_ee @ H_list[i]
        
        return H_ee, H_list

    def calc_forward_kinematics(self,joint_values: list, radians=True):
        dh_table = np.array([
            [0,self.l1,0,pi],
            [self.joint_values[0],-self.l2, 0, 0.5*pi],
            [self.joint_values[1] + 0.5*pi, 0, self.l3,pi],
            [self.joint_values[2] + 0.5*pi, 0, 0, 0.5*pi],
            [self.joint_values[3], -self.l4 - self.l5, 0, -0.5*pi],
            [self.joint_values[4], 0, 0, 0.5*pi],
            [self.joint_values[5] - 0.5*pi, -self.l6 - self.l7, 0, pi]
        ])

        H_ee, H_list = self.dh_to_H(dh_table=dh_table)

        ee = ut.EndEffector()
        ee.x, ee.y, ee.z = (H_ee @ np.array([0, 0, 0, 1]))[:3]

        rpy = ut.rotm_to_euler(H_ee[:3, :3])
        ee.rotx, ee.roty, ee.rotz = rpy[0], rpy[1], rpy[2]

        return ee, H_list

if __name__ == "__main__":
    model = KinovaRobot()
    robot = RobotSim(robot_model=model)
    viz = Visualizer(robot=robot)
    viz.run()