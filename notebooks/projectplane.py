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

#biodegradable

banana = p.loadURDF("notebooks\ycb_assets/011_banana.urdf")


apple = p.loadURDF("notebooks\ycb_assets/013_apple.urdf")
box = p.loadURDF("notebooks\ycb_assets/003_cracker_box.urdf")
strawberry = p.loadURDF("notebooks\ycb_assets/012_strawberry.urdf")
pear = p.loadURDF("notebooks\ycb_assets/016_pear.urdf")

#non biodegradable
tennisball = p.loadURDF("notebooks\ycb_assets/056_tennis_ball.urdf")
mug = p.loadURDF("notebooks\ycb_assets/025_mug.urdf")
soccerball = p.loadURDF("notebooks\ycb_assets/053_mini_soccer_ball.urdf")
airplane = p.loadURDF("notebooks\ycb_assets/072-a_toy_airplane.urdf")
beachcleanser = p.loadURDF("notebooks\ycb_assets/021_bleach_cleanser.urdf")






p.setGravity(0, 0, -60)




while True:
    p.stepSimulation()
    time.sleep(0.01)
p.disconnect()
print("Simulation step completed and disconnected.")