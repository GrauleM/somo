from pathlib import Path
import os

import pybullet as p
import pybullet_data

import time  # for waiting

from pybullet_utils import bullet_client as bc
from pybullet_utils import urdfEditor as ed


p0 = bc.BulletClient(connection_mode=p.DIRECT)
p0.setAdditionalSearchPath(pybullet_data.getDataPath())

p1 = bc.BulletClient(connection_mode=p.DIRECT)
p1.setAdditionalSearchPath(pybullet_data.getDataPath())

# can also connect using different modes, GUI, SHARED_MEMORY, TCP, UDP, SHARED_MEMORY_SERVER, GUI_SERVER

finger_urdf_path = os.path.join(Path(__file__).parent, "abondance_finger.urdf")

finger0 = p0.loadURDF(finger_urdf_path)
finger1 = p1.loadURDF(finger_urdf_path)

ed0 = ed.UrdfEditor()
ed0.initializeFromBulletBody(finger1, p1._client)  # xx todo: switch 0 and 1
ed1 = ed.UrdfEditor()
ed1.initializeFromBulletBody(finger0, p0._client)

parentLinkIndex = 0

jointPivotXYZInParent = [0, 0, 0]
jointPivotRPYInParent = [0, 0, 0]

jointPivotXYZInChild = [0, 0, 0]
jointPivotRPYInChild = [0, 0, 0]

newjoint = ed0.joinUrdf(
    ed1,
    parentLinkIndex,
    jointPivotXYZInParent,
    jointPivotRPYInParent,
    jointPivotXYZInChild,
    jointPivotRPYInChild,
    p0._client,
    p1._client,
)
newjoint.joint_type = p0.JOINT_FIXED

ed0.saveUrdf("combined.urdf")

# now that the urdf is merged, load and simulate it

# add white ground plane for pretty screen grabs and videos
p.setAdditionalSearchPath(
    pybullet_data.getDataPath()
)  # defines the path used by p.loadURDF
planeId = p.loadURDF("plane.urdf")

physicsClient = p.connect(
    p.GUI
)  # p.GUI for graphical, or p.DIRECT for non-graphical version
world_scaling = 1.0

p.setGravity(0, 0, -world_scaling * 9.81)
p.setPhysicsEngineParameter(enableConeFriction=1)
p.setRealTimeSimulation(0)

startPos = [
    0,
    0,
    0,
]  # have to be careful to set this position such that the box and atuator just touch (to replicate experimental condition)
startOr = p.getQuaternionFromEuler([0, 0, 0])
assemblyId = p.loadURDF(
    "combined.urdf",
    basePosition=startPos,
    baseOrientation=startOr,
    physicsClientId=physicsClient,
)


time_step = 0.0001
p.setTimeStep(time_step)
n_steps = 10000

start_time = time.time()
for i in range(n_steps):
    p.stepSimulation()
end_time = time.time()
print(f"execution time: {end_time-start_time}")

p.disconnect()

#
# print(p0._client)
# print(p1._client)
# print("p0.getNumBodies()=", p0.getNumBodies())
# print("p1.getNumBodies()=", p1.getNumBodies())
#
# pgui = bc.BulletClient(connection_mode=pybullet.GUI)
# pgui.configureDebugVisualizer(pgui.COV_ENABLE_RENDERING, 0)
#
# orn = [0, 0, 0, 1]
# ed0.createMultiBody([0, 0, 0], orn, pgui._client)
# pgui.setRealTimeSimulation(1)
#
# pgui.configureDebugVisualizer(pgui.COV_ENABLE_RENDERING, 1)
#
# while (pgui.isConnected()):
#   pgui.getCameraImage(320, 200, renderer=pgui.ER_BULLET_HARDWARE_OPENGL)
#   time.sleep(1. / 240.)
