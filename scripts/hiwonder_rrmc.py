# main.py
"""
Main Application Script
----------------------------
Example code for the MP1 RRMC implementation

Edited to add FiveDOFRobot class that implements calc_velocity_kinematics
"""

import time
from math import cos, sin, pi
import numpy as np
import traceback

from funrobo_hiwonder.core.hiwonder import HiwonderRobot, BaseRobot
from funrobo_kinematics.core.arm_models import FiveDOFRobotTemplate
import funrobo_kinematics.core.utils as ut

class FiveDOFRobot(FiveDOFRobotTemplate):
    def __init__(self):
        super().__init__()
        #hw_robot = HiwonderRobot()
        #self.joint_limits = hw_robot.joint_limits
        self.joint_limits = [[-120, 120],[-90,90],[-120,120],[-100,100],[-90,90],[-120,30]]
        self.joint_limits = [[val[0] * pi / 180, val[1] * pi / 180] for val in self.joint_limits]
        print(f"\n\njoint limits {self.joint_limits}\n\n")
        #del(hw_robot)
    
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
            [self.joint_values[0],self.l1, 0, 0.5 * pi],
            [self.joint_values[1] - 0.5*pi,0,-self.l2,pi],
            [self.joint_values[2],0,-self.l3,pi],
            [self.joint_values[3] + 0.5*pi,0,0,-0.5*pi],
            [self.joint_values[4],self.l4 + self.l5,0,0],
        ])

        H_ee, H_list = self.dh_to_H(dh_table=dh_table)

        ee = ut.EndEffector()
        ee.x, ee.y, ee.z = (H_ee @ np.array([0, 0, 0, 1]))[:3]

        rpy = ut.rotm_to_euler(H_ee[:3, :3])
        ee.rotx, ee.roty, ee.rotz = rpy[0], rpy[1], rpy[2]

        return ee, H_list
    
    def calc_velocity_kinematics(self, joint_values: list, vel: list, dt=0.02):
        """
        Calculates the velocity kinematics for the robot based on the given velocity input.

        Args:
            vel (list): The velocity vector for the end effector [vx, vy, vz].
        """
        new_joint_values = joint_values.copy()

        # move robot slightly out of zeros singularity
        #if all(theta == 0.0 for theta in new_joint_values):
        #    new_joint_values = [theta + np.random.rand()*0.02 for theta in new_joint_values]
        
        # Calculate joint velocities using the inverse Jacobian
        # For this, we don't care about rotation, so we want only a 3 element velocity vector
        #new_joint_values_rad = [val * pi / 180 for val in new_joint_values]
        J = self.calc_jacobians(new_joint_values)
        Jv = J[0:3,:] # Only includes linear velocity, shape (3,5)

        # Shift away from singularities using damped inverse
        lam = 0.05
        JJt = Jv @ Jv.T # shape (3,3)
        JJt = JJt + (lam**2 * np.eye(3))
        J_inv_damped = Jv.T @ self.inv_jacobian(JJt)
        joint_vel = J_inv_damped @ vel
        #joint_vel = joint_vel_rad * 180 / pi 
        #joint_vel = joint_vel_rad

        #print(f"joint vel shape: {joint_vel.shape}, J_inv_damped shape: {J_inv_damped.shape}, thingy shape: {[limit[0] for limit in self.joint_vel_limits]}")
        #print(f"joint vel limits: {self.joint_vel_limits}")
        #print(f"j v limits: {self.joint_vel_limits}")
 
        joint_vel = np.clip(joint_vel, 
                            [limit[0] for limit in self.joint_vel_limits], 
                            [limit[1] for limit in self.joint_vel_limits]
                        )

        print(f"Commanded linear vel: {vel}, resulting joint vel (rad): {joint_vel}")
        #joint_vel = [v * 180 / pi for v in joint_vel]

        # Update the joint angles based on the velocity
        for i in range(self.num_dof):
            new_joint_values[i] += dt * joint_vel[i]

        # Ensure joint angles stay within limits
        print(f"Almost final joint values sent: {new_joint_values}")
        new_joint_values = np.clip(new_joint_values, 
                               [limit[0] for limit in self.joint_limits], 
                               [limit[1] for limit in self.joint_limits]
                            )
        
        print(f"Final joint values sent: {new_joint_values}")
        return new_joint_values

    def calc_jacobians(self,joint_values):
        """
        Calculates Jacobian matrix given joint angles
        """
        # Jacobian will be 6 rows by 5 colums
        k_hat = np.array([0,0,1])
        J = np.zeros(shape=(6,5))

        # Get ee position and H transform matrices
        ee, H_list = self.calc_forward_kinematics(joint_values=joint_values,radians=True)

        pos_ee = np.array([ee.x, ee.y, ee.z])

        H_0_current = np.eye(4)
        for i,H in enumerate(H_list):
            R_i = H_0_current[0:3,0:3]
            d_i = H_0_current[0:3,3]
            
            pos_joint_to_ee = pos_ee - d_i # lever arm of joint to ee
            z_joint = R_i @ k_hat # axis of rotation of joint

            Jv = np.cross(z_joint,pos_joint_to_ee) # cross product is jacobian
            Jw = z_joint

            #print(f"Jv: {Jv}, Jw: {Jw}")

            J[0:3,i] = Jv
            J[3:6,i] = Jw

            H_0_current = H_0_current @ H
        
        #print(f"\nMy Jacobian was: {J}\n")
        return J
    
    def inv_jacobian(self,J):
        """
        Inverts a provided Jacobian matrix using numpy
        """
        return np.linalg.pinv(J)

def main():
    """ Main loop that reads gamepad commands and updates the robot accordingly. """
    try:

        # Initialize components
        robot = HiwonderRobot()
        model = FiveDOFRobot()

        curr_joint_values = None # Initialize to none
        
        control_hz = 20 
        dt = 1 / control_hz
        t0 = time.time()

        while True:
            t_start = time.time()

            if robot.read_error is not None:
                print("[FATAL] Reader failed:", robot.read_error)
                break

            if robot.gamepad.cmdlist:
                cmd = robot.gamepad.cmdlist[-1]

                if cmd.arm_home:
                    robot.move_to_home_position()
                
                if curr_joint_values is None:
                    curr_joint_values = robot.get_joint_values() # deg
                    curr_joint_values_rad = [v * pi / 180 for v in curr_joint_values] # rad

                #curr_joint_values = robot.get_joint_values()

                ### Convert to radians 
                #curr_joint_values = [v * pi / 180 for v in curr_joint_values]

                vel = [cmd.arm_vx, cmd.arm_vy, cmd.arm_vz]
                curr_joint_values_rad = model.calc_velocity_kinematics(curr_joint_values_rad, vel)
                curr_joint_values = [v * 180 / pi for v in curr_joint_values_rad]
                ### Convert to degrees
                #new_joint_values = [v * 180 / pi for v in new_joint_values]

                # set new joint angles
                print(f"Final values sent (deg): {curr_joint_values}")
                robot.set_joint_values(curr_joint_values, duration=dt, radians=False)

            elapsed = time.time() - t_start
            remaining_time = dt - elapsed
            if remaining_time > 0:
                time.sleep(remaining_time)

            
    except KeyboardInterrupt:
        print("\n[INFO] Keyboard Interrupt detected. Initiating shutdown...")
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")
        traceback.print_exc()
    finally:
        robot.shutdown_robot()




if __name__ == "__main__":
    main()


