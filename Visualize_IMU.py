#Script adapted from: https://toptechboy.com/9-axis-imu-lesson-19-vpython-visualization-of-pitch-and-yaw/
from vpython import *
from time import *
import numpy as np
from scipy.spatial.transform import Rotation
import math
import serial
ad=serial.Serial('com4',115200)
sleep(1)

#below function from https://gist.github.com/simbamangu/4c843ca2b4fed57371a743f850b583aa
def eulerVals(q1, q2, q3):
    """
    Convert quaternion values to Euler angles in degrees:
    pitch
    roll
    yaw
    """
    q0 = np.sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)))
    q2sqr = q2 * q2

    t0 = 2 * (q0 * q1 + q2 * q3)
    t1 = 1 - 2 * (q1 * q1 + q2sqr)
    roll = atan2(t0, t1)

    t2 = +2.0 * (q0 * q2 - q3 * q1)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = asin(t2)

    t3 = +2.0 * (q0 * q3 + q1 * q2)
    t4 = +1.0 - 2.0 * (q2sqr + q3 * q3)
    yaw = atan2(t3, t4)
    
    return pitch, roll, yaw

def is_number(n):
    try:
        float(n)   # Type-casting the string to `float`.
                   # If string is not a valid `float`, 
                   # it'll raise `ValueError` exception
    except ValueError:
        return False
    return True

def check_packet_valid(packet):
    if len(packet) < 16:
        return False
    for n in packet:
        if not is_number(n):
            return False
    return True

def with_scalar_component(qs):
    qs_wr = np.asarray([*qs, np.sqrt(1.0 - np.sum(np.multiply(qs, qs)))]) # Real last
    # qs_wr = np.asarray([np.sqrt(1.0 - np.sum(np.multiply(qs, qs))), *qs]) # Real first
    qs_wr[np.isnan(qs_wr)] = 0
    return qs_wr

def rotated_vector(r, v):
    k=r.apply(v)
    return vector(k[0],k[1],k[2])

#setup VPython scene
scene.range=5
toRad=2*np.pi/360
toDeg=1/toRad
scene.forward=vector(1,-1,1)
scene.up=vector(0,0,-1)

scene.width=600
scene.height=600

# xarrow=arrow(shaftwidth=.1, color=color.red,axis=vector(1,0,0))
# yarrow=arrow(shaftwidth=.1, color=color.green,axis=vector(0,1,0))
# zarrow=arrow(shaftwidth=.1, color=color.blue,axis=vector(0,0,1))

frontArrowQuat=arrow(shaftwidth=.1,color=(color.red*0.5),axis=vector(1,0,0))
sideArrowQuat=arrow(shaftwidth=.1,color=(color.green*0.5),axis=vector(0,0,1))
upArrowQuat=arrow(shaftwidth=.1,color=(color.blue*0.5),axis=vector(0,1,0))

accArrow=arrow(shaftwidth=.1,color=(color.yellow*0.5),axis=vector(0,0,1))
magArrow=arrow(shaftwidth=.1,color=(color.purple*0.5),axis=vector(0,0,1))

# frontArrowEuler=arrow(shaftwidth=.1,color=(color.cyan*0.5),axis=vector(1,0,0))
# sideArrowEuler=arrow(shaftwidth=.1,color=(color.cyan*0.5),axis=vector(0,0,1))
# upArrowEuler=arrow(shaftwidth=.1,color=(color.cyan*0.5),axis=vector(0,1,0))

#loop
while (True):
    while (ad.inWaiting()==0):
        pass
    dataPacket=str(ad.readline())
    splitPacket=dataPacket.split("'")[1].split(",")[:-1] #strip off unicode characters and split the string into a list

    #Log data to terminal
    print(str(splitPacket))

    if check_packet_valid(splitPacket):

        #Parse packet
        Q6_1=float(splitPacket[0])
        Q6_2=float(splitPacket[1])
        Q6_3=float(splitPacket[2])
        Q9_1=float(splitPacket[3])
        Q9_2=float(splitPacket[4])
        Q9_3=float(splitPacket[5])
        HeadAcc=float(splitPacket[6])
        AX=float(splitPacket[7])
        AY=float(splitPacket[8])
        AZ=float(splitPacket[9])
        GX=float(splitPacket[10])
        GY=float(splitPacket[11])
        GZ=float(splitPacket[12])
        MX=float(splitPacket[13])
        MY=float(splitPacket[14])
        MZ=float(splitPacket[15])

        #Generate Quaternion
        Q6V = with_scalar_component([Q6_1, Q6_2, Q6_3])
        Q9V = with_scalar_component([Q9_1, Q9_2, Q9_3])
        Q = Rotation.from_quat(Q6V)

        #Use Quaternion to rotate the unit vectors
        v1=rotated_vector(Q, np.array([1,0,0]))
        v2=rotated_vector(Q, np.array([0,1,0]))
        v3=rotated_vector(Q, np.array([0,0,1]))

        #Paint Quaternion unit vectors
        frontArrowQuat.axis=v1
        sideArrowQuat.axis=v2
        upArrowQuat.axis=v3

        # #Use simbamangu's code to generate Euler angles
        # pitch, roll, yaw = eulerVals(Q9_1, Q9_2, Q9_3)
        # QE = Rotation.from_euler('zyx', [yaw, pitch, roll])
        
        # #Paint Euler unit vectors
        # v1E=rotated_vector(QE, np.array([1,0,0]))
        # v2E=rotated_vector(QE, np.array([0,1,0]))
        # v3E=rotated_vector(QE, np.array([0,0,1]))
        # frontArrowEuler.axis=v1E
        # sideArrowEuler.axis=v2E
        # upArrowEuler.axis=v3E

        #Paint abs acceleration and mag vector
        accArrow.axis=vector(AX,AY,AZ)
        accArrow.length = accArrow.length/10000
        magArrow.axis=vector(MX,MY,MZ)
        magArrow.length = magArrow.length/100
        