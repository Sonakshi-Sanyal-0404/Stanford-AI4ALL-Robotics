import cv2
import numpy as np
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
import time
import math

class TopDownCamera:

    def __init__(self, 
                 img_width: int,
                 img_height: int,
                 camera_position: list[float],
                 floor_plane_size: float,
                 ):
        """
        A TopDownCamera for use in the simulation to get images.

        Args:
            img_width: int, width of a gathered image in pixels
            img_height: int, height of a gathered image in pixels
            camera_position: list[float], position of the camera in world coordinates, [X, Y, Z]
            floor_plane_size: float, size of each side of the floor plane in meters (the camera is looking down on this plane)
        """
        self._img_width = img_width
        self._img_height = img_height
        self._floor_plane_size = floor_plane_size
        self._camera_position = camera_position
        
        # Roll, pitch, and yaw in degrees specify how the camera is rotated in the world (this makes it top-down)
        self._roll = 0
        self._pitch = -90
        self._yaw = 90

        camera_target = camera_position.copy()
        camera_target[2] = 0

        # Extrinsics matrix: camera position in world coordinates
        self._view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=camera_target,
            distance=camera_position[2],
            yaw=self._yaw,
            pitch=self._pitch,
            roll=self._roll,
            upAxisIndex=2
        )

        self._aspect_ratio = img_width / img_height
        self._near = 0.01
        self._far = 10
        self._fov = 2 * np.degrees(np.arctan((floor_plane_size/2) / camera_position[2]))  # field of view to cover 1m

        # Intrinsics matrix
        self._projection_matrix = p.computeProjectionMatrixFOV(
            fov=self._fov,
            aspect=self._aspect_ratio,
            nearVal=self._near,
            farVal=self._far
        )

    def get_image(self):
        """
        Takes a RGB top-down image from inside Pybullet.

        Returns:
            rgb_img: np.ndarray, RGB image of shape (img_height, img_width, 3)
        """
        img_arr = p.getCameraImage(
            width=self._img_width,
            height=self._img_height,
            viewMatrix=self._view_matrix,
            projectionMatrix=self._projection_matrix,
        )
        rgba = np.reshape(img_arr[2], (self._img_height, self._img_width, 4))
        rgb_img = rgba[:, :, :3]
        return rgb_img
    
    def get_pixel_world_coords(self, pixel_x: int, pixel_y: int) -> list[float]:
        """
        Converts a pixel coordinate to 3D world coordinates assuming the pixel lies on the floor plane.

        Args:
            pixel_x: int, x-coordinate of the pixel
            pixel_y: int, y-coordinate of the pixel

        Returns:
            world_coords: list[float], 3D world coordinates of the pixel, [X, Y, Z]
        """
        world_x = (pixel_x / self._img_width) * self._floor_plane_size - self._floor_plane_size/2
        world_y = (pixel_y / self._img_height) * self._floor_plane_size - self._floor_plane_size/2
        world_z = 0
        return [world_x, world_y, world_z]

def linear_interpolation(start, goal, steps):
    # Returns an (steps x len(start)) array of intermediate angles
    return np.linspace(start, goal, steps)

def plan_trajectory(kuka_id, start_angles, target_position, target_orientation = None):
    if target_orientation:
        goal_joint_angles = p.calculateInverseKinematics(bodyUniqueId = kuka_id, 
                                                        endEffectorLinkIndex = 6, 
                                                        targetPosition = target_position,
                                                        targetOrientation = target_orientation,)[:7]
                                                        #currentPositions = start_angles)[:7]
    else:
        goal_joint_angles = p.calculateInverseKinematics(bodyUniqueId = kuka_id, 
                                                        endEffectorLinkIndex = 6, 
                                                        targetPosition = target_position,)[:7]
                                                        #currentPositions = start_angles)[:7]
    
    STEPS = 250
    link_state = p.getLinkState(kukaId, 6)
    trajectory = linear_interpolation(link_state[0], target_position, STEPS)
    trajectory_in_joints = [start_angles]
    #print(trajectory)
    for pt in trajectory:
        if target_orientation:
            goal_joint_angles = p.calculateInverseKinematics(bodyUniqueId = kuka_id, 
                                                        endEffectorLinkIndex = 6, 
                                                        targetPosition = pt,
                                                        targetOrientation = target_orientation,
                                                        currentPositions = trajectory_in_joints[-1])[:7]
        else:
            goal_joint_angles = p.calculateInverseKinematics(bodyUniqueId = kuka_id, 
                                                        endEffectorLinkIndex = 6, 
                                                        targetPosition = pt,
                                                        currentPositions = trajectory_in_joints[-1])[:7]
            
        trajectory_in_joints.append(goal_joint_angles)

    #trajectory_in_joints.append(target_position)
    """
    valid_trajectory = []
    for angles in trajectory:
        for i in range(7):
            p.resetJointState(kuka_id, i, angles[i])
        if not p.getContactPoints(kuka_id):
            valid_trajectory.append(angles)
    """

    #return goal_joint_angles
    return trajectory_in_joints

def execute_trajectory(kuka_id, start_angles, target_position, target_orientation = None):
    """
    Uses plan_trajectory to compute the motion path and moves the KUKA arm along it.
    """
    trajectory = plan_trajectory(kuka_id, start_angles, target_position, target_orientation)
    #print(trajectory)
    tolerance = 2.5

    for angles in trajectory:
        print(angles)
        for i in range(7):
            p.setJointMotorControl2(
                bodyUniqueId=kuka_id,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=angles[i],
                force=350
            )

        continue_step = True
        while continue_step:
            num_joints_in_tolerance = 0
            joint_angles = get_robot_joint_angles(kukaId)
            for i in range(7):
                #print(joint_angles[i])
                if abs(joint_angles[i] - angles[i]) <= tolerance:
                    num_joints_in_tolerance += 1

            if num_joints_in_tolerance == 7: continue_step = False
            p.stepSimulation()
            time.sleep(0.01)

            #wait(0.01)

def get_robot_joint_angles(kuka_id):
    angles = []
    for i in range(7):
        angles.append(p.getJointState(kuka_id, i)[0])
    return angles

def grasp(kuka_id, obj_id):
    constraint_id = p.createConstraint(kuka_id, 6, obj_id, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0.05], [0, 0, 0])
    return constraint_id

def off_grasp(constraint_id):
    p.removeConstraint(constraint_id)

def wait(t):
    for _ in range(int(t * 240)):
        time.sleep(1/240)
        p.stepSimulation()

def move_obj(kuka_id, obj_id, obj_pos, target_pos):
    above_pos, behind_target = obj_pos.copy(), target_pos.copy()
    above_pos[2] += 0.25
    behind_target[1] -= 0.5
    execute_trajectory(kuka_id, get_robot_joint_angles(kuka_id), above_pos, down)
    execute_trajectory(kuka_id, get_robot_joint_angles(kuka_id), obj_pos, down)
    constraint = grasp(kuka_id, obj_id)
    execute_trajectory(kuka_id, get_robot_joint_angles(kuka_id), behind_target, down)
    execute_trajectory(kuka_id, get_robot_joint_angles(kuka_id), target_pos, down)
    wait(0.35)
    off_grasp(constraint)
    wait(0.1)
    execute_trajectory(kuka_id, get_robot_joint_angles(kuka_id), above_bins, down)
    wait(0.2)

p.connect(p.GUI)

p.resetSimulation()

p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
print("Loaded PyBullet plane.")

#biodegradable

banana = p.loadURDF("notebooks/ycb_assets/011_banana.urdf", 
                    basePosition = [1, 1, 0.25],
                      globalScaling = 0.5)

"""

apple = p.loadURDF("notebooks/ycb_assets/013_apple.urdf", 
                   basePosition = [2, 0, 0.5],
                      globalScaling = 0.5)


box = p.loadURDF("notebooks/ycb_assets/003_cracker_box.urdf", 
                 basePosition = [3, 1, 0.5],
                      globalScaling = 0.5)


strawberry = p.loadURDF("notebooks/ycb_assets/012_strawberry.urdf", 
                        basePosition = [1 , -1, 0.5],
                      globalScaling = 0.5)


pear = p.loadURDF("notebooks/ycb_assets/016_pear.urdf", 
                  basePosition = [3, -1, 0.5],
                      globalScaling = 0.5)
"""

#non biodegradable
"""
tennisball = p.loadURDF("notebooks/ycb_assets/056_tennis_ball.urdf", 
                        basePosition = [1, 2, 0.3],
                        globalScaling = 0.7)
"""

"""
mug = p.loadURDF("notebooks/ycb_assets/025_mug.urdf", 
                 basePosition = [2, 2, 0.5])

soccerball = p.loadURDF("notebooks/ycb_assets/053_mini_soccer_ball.urdf", 
                        basePosition = [0, -2, 0.5])

airplane = p.loadURDF("notebooks/ycb_assets/072-a_toy_airplane.urdf", 
                      basePosition = [2, -2, 0.5],
                      globalScaling = 0.5)

beachcleanser = p.loadURDF("notebooks/ycb_assets/021_bleach_cleanser.urdf", 
                           basePosition = [2, -1, 0.5],
                           globalScaling = 0.7)
"""


#bins
biobin = p.loadURDF("notebooks/ycb_assets/bin.urdf", 
                    basePosition = [-1.6, -1.3, 0.5], 
                    globalScaling = 5.0,
                    useFixedBase = True)

nonbiobin = p.loadURDF("notebooks/ycb_assets/bin.urdf", 
                       basePosition = [-1.6, 1.3, 0.5], 
                       globalScaling = 5.0,
                       useFixedBase = True)

#arm
kukaId = p.loadURDF("kuka_iiwa/model.urdf", 
                    basePosition=[0,0,0], 
                    useFixedBase=True, 
                    globalScaling = 3.0)

def reset_arm():
    jointPositions = [-0.000000, -0.000000, 0.000000, 1.570793, 0.000000, -1.036725, 0.000001]
    for jointIndex in range(p.getNumJoints(kukaId)):
        p.resetJointState(kukaId, jointIndex, jointPositions[jointIndex])
        p.setJointMotorControl2(kukaId, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex], 0)

def base_arm_pos():
    jointPositions = [math.pi, -0.000000, 0.000000, 1.570793, 0.000000, -1.036725, 0.000001]
    for jointIndex in range(p.getNumJoints(kukaId)):
        p.setJointMotorControl2(kukaId, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex], 0)
    wait(0.05)

reset_arm()

num_joints = p.getNumJoints(kukaId)
print(f"Loaded KUKA with {num_joints} joints.")

p.setGravity(0, 0, -10)

camera = TopDownCamera(
    img_width=1600, 
    img_height=1600, 
    camera_position=[1, 0, 10], 
    floor_plane_size=5.5
)

# 2. Get an image and display it
"""
img = camera.get_image()
plt.imshow(img)
plt.show()
"""

#[0.85, -0.2, 0.75] goes to center kinda
#[0.85, 0.2, 0.95] around the pos the the banana

bio_bin_pos, syn_bin_pos = [-1.5, -1.3, 1.2], [-1.5, 1.3, 1.2]
above_bins = [1.5, 0, 1.7]
#bio_bin_pos, syn_bin_pos = [1.5, -1.3, 1.2], [1.5, 1.3, 1.2]

down = p.getQuaternionFromEuler([0, 1.01 * math.pi, 0])

while True: # put cv here
    img = camera.get_image()
    break
#testing
#execute_trajectory(get_robot_joint_angles(kukaId), bio_bin_pos, kukaId)
#execute_trajectory(get_robot_joint_angles(kukaId), syn_bin_pos, kukaId)

#pick up banana

input("Press any key to continue ")

move_obj(kukaId, banana, [1, 1, 0.3], bio_bin_pos)
#move_obj(kukaId, tennisball, [1, 2, 0.3], syn_bin_pos)
#[0.75, 2, 0.3]

while True:
    p.stepSimulation()
    time.sleep(1/240)

"""
start, target = [0.85, -0.2, 0.75], [1, 1, 0.95]
execute_trajectory(kukaId, get_robot_joint_angles(kukaId), start)
print("at pos 1")
execute_trajectory(kukaId, get_robot_joint_angles(kukaId), target)
print("at pos 2")

while True:
    p.stepSimulation()
    time.sleep(1/240)
p.disconnect()
print("Simulation step completed and disconnected.")
"""