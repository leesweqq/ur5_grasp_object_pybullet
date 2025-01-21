import pybullet as p
import pybullet_data
import math
import time
import random
from collections import namedtuple

class UR5Robotiq85:
    def __init__(self, pos, ori):
        """
        Initialize the UR5 + Robotiq 85 robot arm class.
        :param pos: Base position of the robot arm [x, y, z]
        :param ori: Base orientation of the robot arm [roll, pitch, yaw]
        """
        self.base_pos = pos  # Base position of the robot arm
        self.base_ori = p.getQuaternionFromEuler(ori)  # Base orientation of the robot arm (quaternion representation)
        self.eef_id = 7  # Define the end-effector link ID
        self.arm_num_dofs = 6  # Number of degrees of freedom of the UR5 robot arm
        self.arm_rest_poses = [-1.57, -1.54, 1.34, -1.37, -1.57, 0.0]  # Default joint positions of the robot arm
        self.gripper_range = [0, 0.085]  # Define the range of gripper opening and closing (minimum and maximum)
        self.max_velocity = 3

    def load(self):
        """
        Load the robot arm model and set control parameters.
        """
        self.id = p.loadURDF('./urdf/ur5_robotiq_85.urdf', self.base_pos, self.base_ori, useFixedBase=True)
        self.__parse_joint_info__()  # Get joint information of the robot arm
        self.__setup_mimic_joints__()  # Set up mimic joints for the gripper

    def __parse_joint_info__(self):
        """
        Get joint information of the robot arm, including controllable joints and degree of freedom ranges.
        """
        jointInfo = namedtuple('jointInfo',
                               ['id', 'name', 'type', 'lowerLimit', 'upperLimit', 'maxForce', 'maxVelocity', 'controllable'])
        self.joints = []
        self.controllable_joints = []

        for i in range(p.getNumJoints(self.id)):
            info = p.getJointInfo(self.id, i)
            jointID = info[0]
            jointName = info[1].decode("utf-8")
            jointType = info[2]
            jointLowerLimit = info[8]
            jointUpperLimit = info[9]
            jointMaxForce = info[10]
            jointMaxVelocity = info[11]
            controllable = jointType != p.JOINT_FIXED
            if controllable:
                self.controllable_joints.append(jointID)
            self.joints.append(
                jointInfo(jointID, jointName, jointType, jointLowerLimit, jointUpperLimit, jointMaxForce, jointMaxVelocity, controllable)
            )

        self.arm_controllable_joints = self.controllable_joints[:self.arm_num_dofs]
        self.arm_lower_limits = [j.lowerLimit for j in self.joints if j.controllable][:self.arm_num_dofs]
        self.arm_upper_limits = [j.upperLimit for j in self.joints if j.controllable][:self.arm_num_dofs]
        self.arm_joint_ranges = [ul - ll for ul, ll in zip(self.arm_upper_limits, self.arm_lower_limits)]

    def __setup_mimic_joints__(self):
        """
        Set up mimic joints for the gripper to enable synchronized motion.
        """
        mimic_parent_name = 'finger_joint'
        mimic_children_names = {
            'right_outer_knuckle_joint': 1,
            'left_inner_knuckle_joint': 1,
            'right_inner_knuckle_joint': 1,
            'left_inner_finger_joint': -1,
            'right_inner_finger_joint': -1
        }
        self.mimic_parent_id = [joint.id for joint in self.joints if joint.name == mimic_parent_name][0]
        self.mimic_child_multiplier = {joint.id: mimic_children_names[joint.name] for joint in self.joints if joint.name in mimic_children_names}

        for joint_id, multiplier in self.mimic_child_multiplier.items():
            c = p.createConstraint(self.id, self.mimic_parent_id, self.id, joint_id,
                                   jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0],
                                   parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])
            p.changeConstraint(c, gearRatio=-multiplier, maxForce=100, erp=1)

    def move_arm_ik(self, target_pos, target_orn):
        """
        Use inverse kinematics to calculate joint angles and move the robot arm to the target position.
        :param target_pos: Target position [x, y, z]
        :param target_orn: Target orientation (quaternion)
        """
        joint_poses = p.calculateInverseKinematics(
            self.id, self.eef_id, target_pos, target_orn,
            lowerLimits=self.arm_lower_limits,
            upperLimits=self.arm_upper_limits,
            jointRanges=self.arm_joint_ranges,
            restPoses=self.arm_rest_poses,
        )
        for i, joint_id in enumerate(self.arm_controllable_joints):
            p.setJointMotorControl2(self.id, joint_id, p.POSITION_CONTROL, joint_poses[i], maxVelocity=self.max_velocity)

    def move_gripper(self, open_length):
        """
        Control the gripper to open or close.
        :param open_length: Target width for gripper opening (0 ~ 0.085m)
        """
        open_length = max(self.gripper_range[0], min(open_length, self.gripper_range[1]))
        open_angle = 0.715 - math.asin((open_length - 0.010) / 0.1143)
        p.setJointMotorControl2(self.id, self.mimic_parent_id, p.POSITION_CONTROL, targetPosition=open_angle)

    def get_current_ee_position(self):
        """
        Get the current position of the end-effector.
        """
        eef_state = p.getLinkState(self.id, self.eef_id)
        return eef_state

def update_simulation(steps, sleep_time=0.01):
    """
    Update the simulation by stepping and waiting for a specified time.
    """
    for _ in range(steps):
        p.stepSimulation()
        time.sleep(sleep_time)

def setup_simulation():
    """
    Set up the simulation environment and objects.
    """
    p.connect(p.GUI)
    p.setGravity(0, 0, -9.8)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.loadURDF("plane.urdf")  # Load the ground plane
    table_id = p.loadURDF("table/table.urdf", [0.5, 0, 0], p.getQuaternionFromEuler([0, 0, 0]))  # Load the table
    cube_id2 = p.loadURDF("cube.urdf", [0.5, 0.9, 0.3], p.getQuaternionFromEuler([0, 0, 0]), globalScaling=0.6, useFixedBase=True)

    tray_pos = [0.5, 0.9, 0.6]  # Set the initial position of the tray
    tray_orn = p.getQuaternionFromEuler([0, 0, 0])  # Set the orientation of the tray
    tray_id = p.loadURDF("tray/tray.urdf", tray_pos, tray_orn)  # Load the tray model
    return tray_pos, tray_orn

def random_color_cube(cube_id):
    """
    Randomly set a color for the cube.
    :param cube_id: URDF ID of the cube
    """
    # Randomly generate a color (RGBA)
    color = [random.random(), random.random(), random.random(), 1.0]  # Random color with full opacity
    p.changeVisualShape(cube_id, -1, rgbaColor=color)

def move_and_grab_cube(robot, tray_pos, counter):
    """
    Move the robot arm to the cube's position and grab it.
    """

    while True:
        # Set the initial posture for the robot arm to approach the cube
        target_joint_positions = [0, -1.57, 1.57, -1.5, -1.57, 0.0]
        for i, joint_id in enumerate(robot.arm_controllable_joints):
            p.setJointMotorControl2(robot.id, joint_id, p.POSITION_CONTROL, target_joint_positions[i])
        update_simulation(200)
        x_range = [0.3, 0.7]  # Range for x-coordinate
        y_range = [-0.1, 0.1]  # Range for y-coordinate
        cube_start_pos = [random.uniform(x_range[0], x_range[1]),
                        random.uniform(y_range[0], y_range[1]), 0.65]
        cube_start_pos1=0.78
        cube_start_orn = p.getQuaternionFromEuler([0, 0, 0])  # Cube orientation
        cube_id = p.loadURDF("cube_small.urdf", cube_start_pos, cube_start_orn)
        random_color_cube(cube_id)  # Set a random color

        # Get the position and orientation (quaternion) of the end-effector
        eef_state = p.getLinkState(robot.id, robot.eef_id)

        # eef_state[0] is the position of the end-effector (x, y, z)
        eef_position = eef_state[0]

        # eef_state[1] is the orientation of the end-effector (quaternion)
        eef_orientation = eef_state[1]

        # Display the results
        print("End-effector position:", eef_position)
        print("End-effector orientation (quaternion):", eef_orientation)

        # Move to a position above the cube
        robot.move_arm_ik([cube_start_pos[0], cube_start_pos[1], cube_start_pos1+0.05], eef_orientation)
        time.sleep(0.5)
        update_simulation(50)  # Update the simulation for 50 steps
        # Move to the cube
        robot.move_arm_ik([cube_start_pos[0], cube_start_pos[1], cube_start_pos1], eef_orientation)
        update_simulation(50)
        # Grasp the cube
        robot.move_gripper(0.01)  # Close the gripper
        update_simulation(20)
        # Lift the cube
        robot.move_arm_ik([cube_start_pos[0], cube_start_pos[1], cube_start_pos1 + 0.4], eef_orientation)
        update_simulation(50)
        # Move to a position above the tray
        tray_pos1=random.uniform(0.1, 0.3)
        robot.move_arm_ik([tray_pos[0]+tray_pos1, tray_pos[1]+tray_pos1, tray_pos[2] + 0.56], eef_orientation)
        update_simulation(200)
        # Release the cube
        robot.move_gripper(0.085)  # Open the gripper
        update_simulation(50)

        # Update the displayed counter
        counter += 1  # Increment the counter
        # Update the text displayed above the tray
        p.addUserDebugText(f"Count: {counter}",textColorRGB=[255, 0, 0], textPosition=[tray_pos[0], tray_pos[1], tray_pos[2] + 0.3],
                           textSize=3, lifeTime=10)  

def main():
    tray_pos, tray_orn = setup_simulation()
    
    # Load the robot arm
    robot = UR5Robotiq85([0, 0, 0.62], [0, 0, 0])
    robot.load()

  
    counter = 0
    move_and_grab_cube(robot, tray_pos, counter)

if __name__ == "__main__":
    main()
