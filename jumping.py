import matplotlib.pyplot as plt
import numpy as np
import math as m
from scipy.integrate import odeint
import pybullet as p
import math as m
import time 
import pybullet_data


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #this will load the plane urdf 
p.setGravity(0,0,-10) #along the Z axis
planeId = p.loadURDF("plane.urdf")

#---loading the robotID----:
startPos = [0,0,1] # it will be spawned at z=1
startOrientation = p.getQuaternionFromEuler([0,m.pi/2,0]) #angle at which it will be spawned
robot = p.loadURDF("/home/anushtup/Documents/Coding/Robot_sims/sim_env/Jumping/jumping.urdf",startPos, startOrientation, useFixedBase = True)


#------------------------------------------------------------------------------
p.setGravity(0,0,-10)

jointID = 1
jLow1 = p.getJointInfo(robot, jointID)[8] 
jUp1 = p.getJointInfo(robot, jointID)[9]
jytpe1 = p.getJointInfo(robot, jointID)[2]

x_pos = []
y_pos = []
z_pos = []

x_or = []
y_or = []
z_or = []

for step in range( 1000 ):
    joint_0_targ = np.random.uniform(jLow1, jUp1)
    joint_1_targ = np.random.uniform(jLow1, jUp1)

    p.setJointMotorControlArray(robot, [0,1], p.POSITION_CONTROL, targetPositions = [joint_0_targ, joint_1_targ])
    p.stepSimulation()

    time.sleep(0.05)

    x_pos.append(p.getLinkStates(robot, [1])[0][0][0])
    y_pos.append(p.getLinkStates(robot, [1])[0][0][1])
    z_pos.append(p.getLinkStates(robot, [1])[0][0][1])

    x_or.append(p.getLinkStates(robot, [1])[0][1][0])
    y_or.append(p.getLinkStates(robot, [1])[0][1][1])
    z_or.append(p.getLinkStates(robot, [1])[0][1][2])

    #print(p.getLinkStates(robot, [0,1]))


#-------------------------------------------------------------------------------    

#POSITION CONTROL MODE:-->targetPosition is in radians!
# p.setJointMotorControl2(bodyIndex=robotID, jointIndex=0, controlMode=p.POSITION_CONTROL,
#                         targetPosition=1,force=100)

# p.setJointMotorControl2(bodyIndex=robotID, jointIndex=2, controlMode=p.POSITION_CONTROL,
#                         targetPosition=1,force=100)

##VELOCITY CONTROL MODE:-->targetPosition is in radians!
# for joint in range(1,3):
#     p.setJointMotorControl2(robotID, joint, controlMode=p.VELOCITY_CONTROL,
#                          targetVelocity=-5,force=100)
#------------------------------------------------------------------------------------

#--------CONSTRAINTS-----------------------------------------------------------------

# cid_0 = p.createConstraint(parentBodyUniqueId=robotID,
#                    parentLinkIndex=linkNameToID['base'],
#                    childBodyUniqueId=sphereA,
#                    childLinkIndex=-1,
#                    jointType=p.JOINT_POINT2POINT,
#                    jointAxis=[0, 0, 0],
#                    parentFramePosition=[0, 0, -0.14],
#                    childFramePosition=constraintPosition[0][0])


#------------------------------------------------------------------------------------


p.disconnect()


fig,axs=plt.subplots(6)
axs[0].plot(x_pos)
axs[1].plot(y_pos)
axs[2].plot(z_pos)
axs[3].plot(x_or)
axs[4].plot(y_or)
axs[5].plot(z_or)


axs[0].set_title("X Pos")
axs[1].set_title("y Pos")
axs[2].set_title("z Pos")
axs[3].set_title("X orient")
axs[4].set_title("y orient")
axs[5].set_title("z orient")

plt.show()

    


