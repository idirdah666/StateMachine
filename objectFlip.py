"""objectFlip controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np #for math calculations
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

#defining motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

distanceSensors=[] #Enabling distance senors on the ePuck
for i in range (8):
    distanceSensors.append(robot.getDevice('ps'+str(i)))
    distanceSensors[-1].enable(timestep)

state = "AVOID_OBJECT_1"
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    
    d=[]
    for dist in distanceSensors:
        d.append(dist.getValue())
        
    d=np.asarray(d)

    
    if state == "AVOID_OBJECT_1":   #   go forward until two front sensors 
        leftMotor.setVelocity(6.28) #   detect cube in front of it
        rightMotor.setVelocity(6.28)
        if d[0] > 165.0 and d[7] > 165.0:
            state = "TURN_180"      #   then switch states

    
    if state == "TURN_180":
        # Rotate in place to turn 180 degrees
        leftMotor.setVelocity(6.28)   # Set left wheel forward
        rightMotor.setVelocity(-6.28) # Set right wheel backward
        # Wait for enough time to complete the 180-degree turn 
        robot.step(timestep * 124)  
    
        # After turning, move to the next state
        state = "AVOID_OBJECT_2"
     
    
    elif state == "AVOID_OBJECT_2":
        leftMotor.setVelocity(6.28)
        rightMotor.setVelocity(6.28)
        if d[0] > 190.0 and d[7] > 190.0: # same logic as avoid_object_1, but
            state = "TURN_90"             # instead turns 90 degrees
            
    elif state == "TURN_90":
        # Rotate in place to turn 90 degrees
        leftMotor.setVelocity(6.28)   # Set left wheel forward
        rightMotor.setVelocity(-6.28) # Set right wheel backward
        
        # Wait for enough time to complete the 90-degree turn )
        robot.step(timestep * 50)  
    
        # After turning, move to the next state
        state = "SENSE_LEFT"
        
        
    elif state == "SENSE_LEFT":
        leftMotor.setVelocity(6.28) #  march forward until the left
        rightMotor.setVelocity(6.28) # sensor stops sensing the box to the left
        if d[5] < 80:
            state = "STOP"      # then stop
         
    
    elif state == "STOP":
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
    pass

# Enter here exit cleanup code.
