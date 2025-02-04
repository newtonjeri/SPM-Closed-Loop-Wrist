#! /usr/bin/env python3

import numpy as np
import pybullet as p
import pybullet_data
import time

# Load the SPM model from the provided SDF file
sdf_path = "/home/newtonjeri/ros2_work_ws/Closed-Loop-Wrist-Robot-Arm/src/spm_wrist_description/urdf/spm_wrist.sdf"
p.connect(p.GUI)
p.resetSimulation()
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setPhysicsEngineParameter(enableFileCaching=0)
# Load Assets

p.loadURDF("plane.urdf", [0.0, 0.0, 0.0], [0.0, 0.0 ,0.0, 1.0])
spm_ids = p.loadSDF(sdf_path)  # This returns a list, not a single ID
if not spm_ids:
    raise ValueError(f"Failed to load SDF file: {sdf_path}")

spm_id = spm_ids[0]  # Get the first loaded object

for step in range(300):
    focus_position, _= p.getBasePositionAndOrientation(spm_id)
    # p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=0, cameraPitch=40, cameraTargetPosition=focus_position)
    p.stepSimulation()
    time.sleep(1./10.)