import cv2
import numpy as np
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
import time


p.connect(p.GUI)

p.resetSimulation()

p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
print("Loaded PyBullet plane.")

<<<<<<< HEAD
crackebox = p.loadURDF("011_banana.urdf")
=======
#biodegradable

banana = p.loadURDF("notebooks\ycb_assets/011_banana.urdf", basePosition = [0, 6, 0.5])



apple = p.loadURDF("notebooks\ycb_assets/013_apple.urdf", basePosition = [2, 0, 0.5])

box = p.loadURDF("notebooks\ycb_assets/003_cracker_box.urdf", basePosition = [5, 0, 0.5])
strawberry = p.loadURDF("notebooks\ycb_assets/012_strawberry.urdf", basePosition = [-2 , 0, 0.5])
pear = p.loadURDF("notebooks\ycb_assets/016_pear.urdf", basePosition = [-5, 0, 0.5])

#non biodegradable
tennisball = p.loadURDF("notebooks\ycb_assets/056_tennis_ball.urdf", basePosition = [0, 2, 0.5])
mug = p.loadURDF("notebooks\ycb_assets/025_mug.urdf", basePosition = [0, 4, 0.5])
soccerball = p.loadURDF("notebooks\ycb_assets/053_mini_soccer_ball.urdf", basePosition = [0, -2, 0.5])
airplane = p.loadURDF("notebooks\ycb_assets/072-a_toy_airplane.urdf", basePosition = [0, -5, 0.5])
beachcleanser = p.loadURDF("notebooks\ycb_assets/021_bleach_cleanser.urdf", basePosition = [0, -7, 0.5])






p.setGravity(0, 0, -60)






>>>>>>> 99a69646da296c6bab7348a9d6255857f8e1b362
while True:
    p.stepSimulation()
    time.sleep(0.01)
p.disconnect()
print("Simulation step completed and disconnected.")