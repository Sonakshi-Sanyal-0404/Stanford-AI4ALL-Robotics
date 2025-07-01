import cv2
import numpy as np
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
import time

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

p.connect(p.GUI)

p.resetSimulation()

p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
print("Loaded PyBullet plane.")

#biodegradable

banana = p.loadURDF("notebooks/ycb_assets/011_banana.urdf", 
                    basePosition = [1, 1, 0.5])



apple = p.loadURDF("notebooks/ycb_assets/013_apple.urdf", 
                   basePosition = [2, 0, 0.5])

box = p.loadURDF("notebooks/ycb_assets/003_cracker_box.urdf", 
                 basePosition = [3, 1, 0.5])

strawberry = p.loadURDF("notebooks/ycb_assets/012_strawberry.urdf", 
                        basePosition = [1 , -1, 0.5])

pear = p.loadURDF("notebooks/ycb_assets/016_pear.urdf", 
                  basePosition = [3, -1, 0.5])

#non biodegradable
tennisball = p.loadURDF("notebooks/ycb_assets/056_tennis_ball.urdf", 
                        basePosition = [0, 2, 0.5])

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

#bins
biobin = p.loadURDF("notebooks/ycb_assets/bin.urdf", 
                    basePosition = [-2, -2, 0.7], 
                    globalScaling =5.0,
                    useFixedBase = True)

nonbiobin = p.loadURDF("notebooks/ycb_assets/bin.urdf", 
                       basePosition = [-2, 2, 0.7], 
                       globalScaling = 5.0,
                       useFixedBase = True)

#arm
kukaId = p.loadURDF("kuka_iiwa/model.urdf", 
                    basePosition=[0,0,0], 
                    useFixedBase=True, 
                    globalScaling = 3.0)
num_joints = p.getNumJoints(kukaId)
print(f"Loaded KUKA with {num_joints} joints.")

p.setGravity(0, 0, -10)

camera = TopDownCamera(
    img_width=400, 
    img_height=400, 
    camera_position=[1, 0, 10], 
    floor_plane_size=5.5
    )

# 2. Get an image and display it
img = camera.get_image()
plt.imshow(img)
plt.show()

while True:
    img = camera.get_image()
    break # temp

while True:
    p.stepSimulation()
    time.sleep(0.01)
p.disconnect()
print("Simulation step completed and disconnected.")