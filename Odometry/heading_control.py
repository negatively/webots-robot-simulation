# Nama : Yoga Mileniandi
# NIM  : 20/460883/SV/17964

from controller import Robot, Motor, PositionSensor
import math

# variabel yang dibutuhkan
TIME_STEP = 64
MAX_SPEED = 6.28
WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.052
# create the Robot instance.
robot = Robot()

# initialize devices

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

leftSensor = robot.getDevice('left wheel sensor')
rightSensor = robot.getDevice('right wheel sensor')
leftSensor.enable(TIME_STEP)
rightSensor.enable(TIME_STEP)

position_values = [0, 0]

def odometry(left_position, right_position):
    # mendapatkan nilai heading dan distance robot sekarang
    l = left_position.getValue()
    r = right_position.getValue()
    dl = l * WHEEL_RADIUS
    dr = r * WHEEL_RADIUS
    
    distance = (dr + dl) / 2
    heading = (dr - dl) / AXLE_LENGTH
    
    return (distance, heading)

def target(init, pos):
    # mendapatkan nilai heading dan distance yang dituju
    x_init, y_init = init
    x_pos, y_pos = pos 
    
    x_current = x_pos - x_init
    y_current = y_pos - y_init
    
    distance = (x_current**2 + y_current**2) ** 0.5
    heading = math.atan(y_current / x_current)      
   
    return (distance, heading)
    

# feedback loop: step simulation until receiving an exit event
while robot.step(TIME_STEP) != -1:
    
    d_now, head_now = odometry(leftSensor, rightSensor)
    d_tar, head_tar = target((0,0),(-0.5,0.5))
    
                
    if (abs(head_now) < abs(head_tar)):        
        if(head_tar > 0):
            leftMotor.setVelocity(-MAX_SPEED*0.1)
            rightMotor.setVelocity(MAX_SPEED*0.1)
        elif(head_tar < 0):
            leftMotor.setVelocity(MAX_SPEED*0.1)
            rightMotor.setVelocity(-MAX_SPEED*0.1)    
    elif (d_now < d_tar): 
        leftMotor.setVelocity(MAX_SPEED*0.5)
        rightMotor.setVelocity(MAX_SPEED*0.5)        
    else:
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
    print(f"Heading Now : {head_now}")
    print(f"Heaidng Target : {head_tar}")
    print(f"Distance Now : {d_now}")
    print(f"Distance Target : {d_tar}")
     

    



    
    
    