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
banana = p.loadURDF("notebooks\ycb_assets/011_banana.urdf")
apple = p.loadURDF("notebooks\ycb_assets/013_apple.urdf")
box = p.loadURDF("notebooks\ycb_assets/003_cracker_box.urdf")
strawberry = p.loadURDF("notebooks\ycb_assets/012_strawberry.urdf")
pear = p.loadURDF("notebooks\ycb_assets/016_pear.urdf")



p.setGravity(0, 0, -60)







=======
crackerbox = p.loadURDF("ycb_assets/011_banana.urdf")
>>>>>>> ef34fec0bca3ea1a6580c849b396a4ed0087b076

while True:
    p.stepSimulation()
    time.sleep(0.01)
p.disconnect()
print("Simulation step completed and disconnected.")