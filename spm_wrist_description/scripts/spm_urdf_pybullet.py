#! /usr/bin/env python3

import numpy as np
import pybullet as p
import pybullet_data
import time

# Load the SPM model from the provided URDF file
urdf_path = "spm_wrist_description/urdf/spm_wrist.urdf"

# Connect to PyBullet
p.connect(p.GUI)
p.resetSimulation()
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load Assets
p.loadURDF("plane.urdf", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
spm_id = p.loadURDF(urdf_path, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], useFixedBase=True)
obj_of_focus = spm_id

# Get link indices from their names
num_joints = p.getNumJoints(spm_id)
link_name_to_index = {p.getJointInfo(spm_id, i)[12].decode("utf-8"): i for i in range(num_joints)}

# Define the closed-loop joints
closed_loop_joints = [
    {
        "name": "Platform_Joint2",
        "parent": "Platform",
        "child": "PlatformSupportLink2",
        "axis": [0, 0, 1],
        "parent_frame_position": [0.03879, 0.03167, 0.01419],
        "parent_fram_orientation":  p.getQuaternionFromEuler([0.0, -np.pi/3, 0.0]),
        "child_frame_position": [0, 0, 0],
        # "child_frame_orientation": [0.0, 0.0, 0.0, 1.0],
        "joint_range": [-np.pi, np.pi],
    },
    # {
    #     "name": "Platform_Joint3",
    #     "parent": "Platform",
    #     "child": "PlatformSupportLink3",
    #     "axis": [0, 0, 1],
    #     "parent_frame_position": [-0.007013, 0.03142, -0.04085],
    #     "child_frame_position": [0, 0, 0],
    #     "joint_range": [-np.pi, np.pi],
    # },
]

# Add constraints for closed-loop joints
for joint_info in closed_loop_joints:
    parent_link_index = link_name_to_index[joint_info["parent"]]
    child_link_index = link_name_to_index[joint_info["child"]]

    # Create a constraint to simulate the revolute joint
    constraint_id = p.createConstraint(
        parentBodyUniqueId=spm_id,
        parentLinkIndex=parent_link_index,
        childBodyUniqueId=spm_id,
        childLinkIndex=child_link_index,
        jointType=p.JOINT_POINT2POINT,
        jointAxis=joint_info["axis"],
        parentFramePosition=joint_info["parent_frame_position"],
        # parentFrameOrientation=joint_info["parent_frame_orientation"],
        childFramePosition=joint_info["child_frame_position"],
        # jointChildFrameOrientation=joint_info["child_frame_orientation"],
    )

    # Set joint limits (optional)
    p.changeConstraint(constraint_id, maxForce=100)

# Simulation loop
for step in range(500):
    focus_position, _ = p.getBasePositionAndOrientation(spm_id)
    # p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=0, cameraPitch=40, cameraTargetPosition=focus_position)
    p.stepSimulation()
    time.sleep(1.0 / 10.0)

# Disconnect from PyBullet
p.disconnect()