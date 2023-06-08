import pybullet as p
import time
import numpy as np
import matplotlib.pyplot as plt

# physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version

# p.setAdditionalSearchPath(pybullet_data.getDataPath()) #this will load the plane urdf 
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW,0)
p.createCollisionShape(p.GEOM_PLANE)
plId=p.createMultiBody(0,0)
p.resetDebugVisualizerCamera( cameraDistance=4, cameraYaw=10, cameraPitch=-20, 
                              cameraTargetPosition=[0.0, 0.0, 0.25])
p.setGravity(0,0,-10) #along the Z axis

# planeId = p.loadURDF("plane.urdf")

#---loading the bodyId----:
footIdos = [0,0,1] # it will be spawned at z=1
startOrientation = p.getQuaternionFromEuler([0,0,0]) #angle at which it will be spawned


#creating the robot:
sh_colFoot = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.01,0.01,0.1])
sh_colBody = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.2,0.2,0.1])
sh_colBody = p.createCollisionShape(p.GEOM_CYLINDER,radius=0.13, height=0.6)
sh_visBody = p.createVisualShape(p.GEOM_CYLINDER,radius=0.13, length=0.6, rgbaColor=[0.4,0.4,0.5,1])
sh_colPx = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.01,0.1,0.1])
sh_colPy = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.1,0.01,0.1])

bodyId=p.createMultiBody(baseMass=1,baseCollisionShapeIndex = sh_colBody, baseVisualShapeIndex = sh_visBody,
                        basePosition = [0,0,1.5],baseOrientation=[0,0,0,1])
footId=p.createMultiBody(baseMass=1,baseCollisionShapeIndex = sh_colFoot,
                        basePosition = [0,0,0.5],baseOrientation=[0,0,0,1])

# base = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.1,0.1,0.1])
# sh_colFoot = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.01,0.01,0.1])
# sh_colPx = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.01,0.1,0.1])
# sh_colPy = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.1,0.01,0.1])
# sh_colBody = p.createCollisionShape(p.GEOM_BOX,halfExtents=[0.2,0.2,0.1])
# Body_1 = p.createCollisionShape(p.GEOM_CYLINDER,radius=0.13, height=0.6)
# sh_visBody = p.createVisualShape(p.GEOM_CYLINDER,radius=0.13, length=0.6, rgbaColor=[0.4,0.4,0.5,1])


# bodyId=p.crefootIdtiBody(baseMass=1,baseCollisionShapeIndex = Body_1,baseVisualShapeIndex = sh_visBody,
#                         basePosition = [0,0,1.5],baseOrientation=[0,0,0,1])

# footID=p.createMultiBody(baseMass = 1,baseCollisionShapeIndex = sh_colFoot, 
#                           basePosition = [0,0,0.5],baseOrientation=startOrientation)

# #----------------------INERTIA INCREASING PLATES-----------------------
# cubeId3=p.createMultiBody(baseMass = 3,baseCollisionShapeIndex = sh_colPx,
#                         basePosition = [-0.5,0,1.5],baseOrientation=[0,0,0,1])
# cubeId4=p.createMultiBody(baseMass = 3,baseCollisionShapeIndex = sh_colPx,
#                         basePosition = [0.5,0,1.5],baseOrientation=[0,0,0,1])
# cubeId5=p.createMultiBody(baseMass = 3,baseCollisionShapeIndex = sh_colPy,
#                         basePosition = [0,-0.5,1.5],baseOrientation=[0,0,0,1])
# cubeId6=p.createMultiBody(baseMass = 3,baseCollisionShapeIndex = sh_colPy,
#                          basePosition = [0,0.5,1.5],baseOrientation=[0,0,0,1])
cubeId3=p.createMultiBody(baseMass=3,baseCollisionShapeIndex = sh_colPx,
                        basePosition = [-0.5,0,1.5],baseOrientation=[0,0,0,1])
cubeId4=p.createMultiBody(baseMass=3,baseCollisionShapeIndex = sh_colPx,
                        basePosition = [0.5,0,1.5],baseOrientation=[0,0,0,1])
cubeId5=p.createMultiBody(baseMass=3,baseCollisionShapeIndex = sh_colPy,
                        basePosition = [0,-0.5,1.5],baseOrientation=[0,0,0,1])
cubeId6=p.createMultiBody(baseMass=3,baseCollisionShapeIndex = sh_colPy,
                        basePosition = [0,0.5,1.5],baseOrientation=[0,0,0,1])

#Scenery e.g. an inclined box
boxHalfLength = 2.5
boxHalfWidth = 2.5
boxHalfHeight = 0.2
sh_colBox = p.createCollisionShape(p.GEOM_BOX,halfExtents=[boxHalfLength,boxHalfWidth,boxHalfHeight])
sh_visBox = p.createVisualShape(p.GEOM_BOX,halfExtents=[boxHalfLength,boxHalfWidth,boxHalfHeight], rgbaColor=[0,0,0,1])

block=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [-2,0,-0.1],baseOrientation=[0.0,0.1,0.0,1])
sth=0.15
block2=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox, baseVisualShapeIndex = sh_visBox,
                        basePosition = [5.75,0.15,-0.2+1*sth],baseOrientation=[0.0,0.0,0.0,1])
block3=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [5.75+0.33,0,-0.2+2*sth],baseOrientation=[0.0,0.0,0.0,1])
block4=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [5.75+0.66,0.2,-0.2+3*sth],baseOrientation=[0.1,0.0,0.0,1])
block5=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_colBox,
                        basePosition = [5.75+0.99,0.1,-0.2+4*sth],baseOrientation=[0.0,-0.1,0.0,1])

box11l=0.5
box11w=0.5
box11h=0.1
sh_box11 = p.createCollisionShape(p.GEOM_BOX,halfExtents=[box11l,box11w,box11h])
sth=0.15
for k in range(10):
    p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_box11,
                       basePosition = [3+0.4*k,-1+k/200,k*sth],baseOrientation=[0.0,0.0,0.0,1])
p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_box11,
                  basePosition = [3+0.4*10,-1,k*sth+0.01],baseOrientation=[0.0,0.0,0.0,1])
for k in range(10):
    p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_box11,
                       basePosition = [3+0.4*10+k/200,-0.5+0.4*k,(k+10)*sth],baseOrientation=[0.0,0.0,0.0,1])
box14_1l=7
box14_1w=0.75
box11h=0.1
sh_box14_1 = p.createCollisionShape(p.GEOM_BOX,halfExtents=[box14_1l,box14_1w,box11h])
box14_1=p.createMultiBody(baseMass=0,baseCollisionShapeIndex = sh_box14_1,
                  basePosition = [-0.3,3.1,1.4],baseOrientation=[0.0,-0.1,0.0,1])



p.setGravity(0,0,-10)
p.setRealTimeSimulation(1)
#make to plane less slippery
p.changeDynamics(plId,-1,lateralFriction=10)
p.changeDynamics(block5,-1,lateralFriction=10)
p.changeDynamics(box14_1,-1,lateralFriction=10)

#connections:
cid_0= p.createConstraint(bodyId, -1 ,footId, -1 , p.JOINT_FIXED, jointAxis=[0,0,0],parentFramePosition=[0,0,0], childFramePosition=[0,0,1])
cid4 = p.createConstraint(bodyId,-1,cubeId3,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0.25,0,0])
cid5 = p.createConstraint(bodyId,-1,cubeId4,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[-0.25,0,0])
cid6 = p.createConstraint(bodyId,-1,cubeId5,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,0.25,0])
cid7 = p.createConstraint(bodyId,-1,cubeId5,-1,p.JOINT_FIXED,[0,0,0],[0,0,0],[0,-0.25,0])


# #simple simulation to start:
# p.setTimeStep(0.001)
# p.setRealTimeSimulation(1)


#initiate:
pivot=[0,0,0,1]
decomprPhase=0

Joints=p.getNumJoints(bodyId)
CubePos, cubeOrn = p.getBasePositionAndOrientation(bodyId)
Des= cubeOrn
Euler=p.getEulerFromQuaternion(cubeOrn)

t=0
tstr=0
vx=0
vy=0
zgnd=0
jmp=0
xgl=7
ygl=-1


destination=[]
x_vel=[]
y_vel=[]
z_vel=[]
z_pos=[]
x_pos=[]
y_pos=[]

1
# while 1:
for i in range(1000):
    p.resetDebugVisualizerCamera( cameraDistance=6, cameraYaw=-130+t/10, cameraPitch=-60, 
                              cameraTargetPosition=[cubePos[0], cubePos[1], 0.25])
    t+=1    
    time.sleep(0.01)

    keys = p.getKeyboardEvents()
    if keys.get(65297): #Up
        vx+=0.002
    if keys.get(65298): #Down
        vx-=0.002
    if keys.get(65296): #Right
        vy-=0.002
    if keys.get(65295): #Left
        vy+=0.002
    if keys.get(97):   #A
        if jmp==0:        
            vx*=3
            vy*=3
        jmp=1
    
    if cubePos[0]<xgl:
        vx=+0.08
    else:
        vx=-0.08
        if xgl==-8 and cubePos[0]<6.5:
            vx=-0.04
        
        ygl=3.1
    if cubePos[1]<ygl:
        vy=+0.08
        if xgl==-8 and cubePos[0]<6.5:
            vy=+0.04
    else:
        vy=-0.08
        if ygl>3:
            xgl=-8
        if xgl==-8 and cubePos[0]<6.5:
            vy=-0.04
    
    #computing positions velocities, orientation angles, etc
    cube_prev=cubePos
    Euler_prev=Euler
    cubePos, cubeOrn = p.getBasePositionAndOrientation(bodyId)
    vel_x_cube_pos=(cubePos[0]-cube_prev[0])/0.01
    vel_y_cube_pos=(cubePos[1]-cube_prev[1])/0.01
    vel_z_cube_pos=(cubePos[2]-cube_prev[2])/0.01
    x_vel.append(vel_x_cube_pos)
    y_vel.append(vel_y_cube_pos)
    z_vel.append(vel_z_cube_pos)
    z_pos.append(cubePos[2])
    x_pos.append(cubePos[0])
    y_pos.append(cubePos[1])

    Euler=p.getEulerFromQuaternion(cubeOrn)
    omega_x=(Euler[0]-Euler_prev[0])/0.01
    omega_y=(Euler[1]-Euler_prev[1])/0.01

    x_foot, dum=p.getBasePositionAndOrientation(footId)
    if (vel_z_cube_pos>0 and decomprPhase==0):
        decomprPhase=1
        tstr=t    
        zgnd=x_foot[2]+0.05

    if (x_foot[2]-zgnd>0.105 and decomprPhase==1):
        decomprPhase=2
        if jmp==1:
            vx=vx/3
            vy=vy/3
        jmp=0
    
    if x_foot[2]-zgnd<0.105 and decomprPhase==2:
        decomprPhase=0

    if decomprPhase==1:
        #decompressing: PD control on orientation of body during stance
        DesEU=p.getEulerFromQuaternion(Des)
        Des = p.getQuaternionFromEuler(DesEU + np.array([-0.07*omega_x-0.3*Euler[0]   -0.15*(-(vel_y_cube_pos-vy)*np.cos(Euler[2])+(vel_x_cube_pos-vx)*np.sin(Euler[2])),
                                                             -0.07*omega_y-0.3*Euler[1]   -0.15*( (vel_x_cube_pos-vx)*np.cos(Euler[2])+(vel_y_cube_pos-vy)*np.sin(Euler[2])),0.0]))
        if ((t-tstr)<8 and jmp==1):  #trust for a small time interval (increased spring force)
            p.changeConstraint(cid_0,pivot,jointChildFrameOrientation=Des, maxForce=1300)
        elif ((t-tstr)<8):
            p.changeConstraint(cid_0,pivot,jointChildFrameOrientation=Des, maxForce=600)
        else:
            p.changeConstraint(cid_0,pivot,jointChildFrameOrientation=Des, maxForce=300)
    else:
        #flight and compression: Reposition foot for next landing based on body horizontal velocity and orientation
        if (x_foot[2]-zgnd>0.105):
            Des = p.getQuaternionFromEuler(
                    [+0.15*(-(vel_y_cube_pos-0.0)*np.cos(Euler[2])+(vel_x_cube_pos-0.0)*np.sin(Euler[2])) + Euler[0],
                     +0.15*( (vel_x_cube_pos-0.0)*np.cos(Euler[2])+(vel_y_cube_pos-0.0)*np.sin(Euler[2])) + Euler[1], 0.0])
            
        p.changeConstraint(cid_0,pivot,jointChildFrameOrientation=Des, maxForce=300)
    destination.append(Des)

p.disconnect()


xrot=[]
yrot=[]
zrot=[]
for i in range(len(destination)):
    print(p.getEulerFromQuaternion(destination[i]))
    xrot.append(destination[i][0])
    yrot.append(destination[i][1])
    zrot.append(destination[i][2])

fig,axs=plt.subplots(7)
axs[0].plot(xrot)
axs[1].plot(yrot)
axs[2].plot(zrot)
axs[3].plot(x_vel)
axs[4].plot(y_vel)
axs[5].plot(z_vel)
axs[6].plot(z_pos)

axs[0].set_title("X rot Euler")
axs[1].set_title("y rot Euler")
axs[2].set_title("z rot Euler")
axs[3].set_title("X vel")
axs[4].set_title("y vel")
axs[5].set_title("z vel")
axs[6].set_title("z pos")


# plt.plot(destination)
plt.show()

