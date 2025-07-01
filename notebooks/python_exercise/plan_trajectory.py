"""
Goal: Motion planning & grasping

Assume you know where an object is and what the object type is
Use inverse kinematics to move robot arm to that position, grasp it (we will discuss how to do this), and then move to above one of the bin positions in a collision free manner (motion planning to avoid obstacles)
def plan_trajectory(start_angles, object_target_position, is_object_biodegradable)
Will learn how to grasp later
Grasping: tutorial-pybullet-pick-place-virtual-suction.ipynb
1) Use IK to plan a path to the object
2) Use IK to plan a path to where you are dropping it off
Two possible target locations (assume know where they are)
1) biodegradable bin
2) synthetic bin
Need pybullet for this
In presentation:
Mention we are using suction as the method of grasping
We assume all objects are sufficiently flat or rigid
#

def linear_interpolation(start, goal, steps):
    # Returns an (steps x len(start)) array of intermediate angles
    return np.linspace(start, goal, steps)

def plan_trajectory(start_angles, target_position):
    
    #Plan a joint-space trajectory from start_angles to reach target_position (x,y,z).
    #TODO: Implement motion planning (e.g., linear interpolation + IK).
    
    # Placeholder: return an empty list for now
    return []
"""


"""
Prerequisites
    kuka_id must be defined
    p = pybullet
    np = numpy
    time = time
"""
def linear_interpolation(start, goal, steps):
    # Returns an (steps x len(start)) array of intermediate angles
    return np.linspace(start, goal, steps)

def plan_trajectory(start_angles, target_position, kuka_id):
    goal_joint_angles = p.calculateInverseKinematics(kuka_id, 6, target_position)[:7]
    steps = 50
    trajectory = linear_interpolation(start_angles, goal_joint_angles, steps)

    valid_trajectory = []
    for angles in trajectory:
        for i in range(7):
            p.resetJointState(kuka_id, i, angles[i])
        if not p.getContactPoints(kuka_id):
            valid_trajectory.append(angles)

    return valid_trajectory

def execute_trajectory(start_angles, target_position, kuka_id):
    """
    Uses plan_trajectory to compute the motion path and moves the KUKA arm along it.
    """
    trajectory = plan_trajectory(start_angles, target_position)

    for angles in trajectory:
        for i in range(7):
            p.setJointMotorControl2(
                bodyUniqueId=kuka_id,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=angles[i]
            )
        p.stepSimulation()
        time.sleep(0.01)



