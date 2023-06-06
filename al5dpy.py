# 2017-07-12 by scharette modified by sssynk
# 2D ik

from math import sqrt, atan, acos, fabs, atan2, degrees

# Define legnth of AL5 sections
# if AL5A
#A = 3.75;
#B = 4.25;

#if AL5B
#A = 4.75;
#B = 5.00;

#if AL5D
A = 5.75;
B = 7.375;

# Constants
rtod = 57.295779	# Radians to degrees constant
CST_ANGLE_MIN = 0
CST_ANGLE_MAX = 180
CST_RC_MIN = 500
CST_RC_MAX = 2500

def ard_constrain(value, min, max):
    if(value < min):
        value = min
    if(value > max):
        value = max
    return (value)

def ard_map(value, min1, max1, min2, max2):
    value = ((value - min1) * (max2 - min2) / (max1 - min1) + min2);
    return (value)

def getPulseFromAngle(angle):
    angle = ard_constrain(angle, CST_ANGLE_MIN, CST_ANGLE_MAX)
    pulse = ard_map(angle, CST_ANGLE_MIN, CST_ANGLE_MAX, CST_RC_MIN, CST_RC_MAX)
    return (int(pulse));

#def al5_2D_IK(targetX, targetY, targetZ, targetGrip, targetWAgl, targetWRot):
def al5_2D_IK(targetXYZGWAWR):

    # Initialize variables
    x = targetXYZGWAWR[0]
    y = targetXYZGWAWR[1]
    z = targetXYZGWAWR[2]
    g = targetXYZGWAWR[3]
    wa = targetXYZGWAWR[4]
    wr = targetXYZGWAWR[5]
    Elbow = 0
    Shoulder = 0
    Wrist = 0
    
    # Get distance
    floatM = sqrt((y * y) + (x * x))
#	print("floatM        = " + str(floatM))
    
    # Check X position for error
    if(floatM <= 0):
        return 1
    
    # Get first angle (radians)
    floatA1 = atan(y / x)
#	print("floatA1       = " + str(floatA1))
#	print("x             = " + str(x))
    
    # Check X position for error
    if(x <= 0):
        return 2
    
    # Get 2nd angle (radians)
    floatA2 = acos((A * A - B * B + floatM * floatM) / ((A * 2) * floatM))
#	print("floatA2       = " + str(floatA2))
    
    # Calculate elbow angle (radians)
    floatElbow = acos((A * A + B * B - floatM * floatM) / ((A * 2) * B))
#	print("floatElbow    = " + str(floatElbow))
    
    # Calculate shoulder angle (radians)
    floatShoulder = floatA1 + floatA2
#	print("floatShoulder = " + str(floatShoulder))
    
    # Obtain angles for shoulder / elbow
    Elbow = floatElbow * rtod
#	print("Elbow         = " + str(floatA2))
    Shoulder = floatShoulder * rtod
#	print("Shoulder      = " + str(Shoulder))
    
    # Check elbow/shoulder angle for error
    if((Elbow <= 0) or (Shoulder <= 0)):
        return 3
    Wrist = fabs(wa - Elbow - Shoulder) - 100
    
    # Return the new values
    motors_SEWBZWrG = (Shoulder, Elbow, Wrist, z, g, wr)
    
    # <<< debug <<<
    #print("SHOULDER\tELBOW   \tWRIST-A \tBASE-ROT\tGRIPPER \tWRIST-R \t")
    #print(str(Shoulder) + "\t" + str((180 - Elbow)) + "\t" + str((180 - Wrist)) + "\t" + str(z) + "\t" + str(g) + "\t" + str(wr) + "\t")
    #print(str(getPulseFromAngle(Shoulder)) + "\t" + str(getPulseFromAngle(180 - Elbow)) + "\t" + str(getPulseFromAngle(180 - Wrist)) + "\t" + str(getPulseFromAngle(z)) + "\t" + str(getPulseFromAngle(g)) + "\t" + str(getPulseFromAngle(wr)) + "\t")
    # >>> debug >>>
    
    return (motors_SEWBZWrG)

def al5_3D_IK(targetXYZGWAWR):
    x = targetXYZGWAWR[0]
    y = targetXYZGWAWR[1]
    z = targetXYZGWAWR[2]
    g = targetXYZGWAWR[3]
    wa = targetXYZGWAWR[4]
    wr = targetXYZGWAWR[5]
    
    Elbow = 0
    Shoulder = 0
    Wrist = 0
    Base = 0
    
    # Get distance of the wrist in the XY plane
    floatM = sqrt((y * y) + (x * x))
    
    # Get angle formed by the line that goes from the robot base to the wrist
    floatA = atan2(z, floatM)
    
    # Get distance of the wrist from the robot base
    floatR = sqrt((z * z) + (floatM * floatM))
    
    # Check position for error
    if(floatR <= 0):
        return 1
    
    # Get first angle of the triangle formed by the wrist, robot base, and link A
    floatA1 = acos((A * A - B * B + floatR * floatR) / (2 * A * floatR))
    
    # Calculate shoulder angle
    floatShoulder = floatA + floatA1
    
    # Calculate elbow angle
    floatElbow = acos((A * A + B * B - floatR * floatR) / (2 * A * B))
    
    # Calculate base angle
    floatBase = atan2(y, x)

    # Obtain angles in degrees
    Elbow = degrees(floatElbow)
    Shoulder = degrees(floatShoulder)
    Base = degrees(floatBase)

    # Check calculated angles for errors
    if((Elbow <= 0) or (Shoulder <= 0) or (Base <= 0)):
        return 3

    Wrist = abs(wa - Elbow - Shoulder) - 100

    # Return the new values
    motors_BSEWBZWrG = (Base, Shoulder, Elbow, Wrist, z, g, wr)

    return (motors_BSEWBZWrG)


def al5_moveMotors(motors_SEWBZWrG, speed_SEWBZWrG, serial):
    
    # Get values from angles to pulses (µs)
    pulseShoulder = getPulseFromAngle(motors_SEWBZWrG[0])
    pulseElbow = getPulseFromAngle((180 - motors_SEWBZWrG[1]))
    pulseWrist = getPulseFromAngle((180 - motors_SEWBZWrG[2]))
    pulseZ = getPulseFromAngle(motors_SEWBZWrG[3])
    pulseG = getPulseFromAngle(motors_SEWBZWrG[4])
    pulseWR = getPulseFromAngle(motors_SEWBZWrG[5])
    
    # Get values from speeds
    speedShoulder = speed_SEWBZWrG[0]
    speedElbow = speed_SEWBZWrG[1]
    speedWrist = speed_SEWBZWrG[2]
    speedZ = speed_SEWBZWrG[3]
    speedG = speed_SEWBZWrG[4]
    speedWR = speed_SEWBZWrG[5]

    # Create the command packet
    commandPacket = ""
    commandPacket += "#0 P" + str(pulseZ) + " S" + str(speedZ) + " "
    commandPacket += "#1 P" + str(pulseShoulder) + " S" + str(speedShoulder) + " "
    commandPacket += "#2 P" + str(pulseElbow) + " S" + str(speedElbow) + " "
    commandPacket += "#3 P" + str(pulseWrist) + " S" + str(speedWrist) + " "
    commandPacket += "#4 P" + str(pulseG) + " S" + str(speedG) + " "
    commandPacket += "#5 P" + str(pulseWR) + " S" + str(speedWR) + "\r"

    # Write the command packet to SSC-32, added to same packet to make motors move at the same time
    serial.write(commandPacket.encode())
    
    return
