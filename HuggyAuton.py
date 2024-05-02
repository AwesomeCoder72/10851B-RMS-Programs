# VEX IQ Python B Project
import vex
import sys
import motor_group

#region config
brain      = vex.Brain()
gyro       = vex.Gyro(vex.Ports.PORT1) 
armRight   = vex.Motor(vex.Ports.PORT2) 
driveH     = vex.Motor(vex.Ports.PORT4, True)  # Reverse Polarity
driveRight = vex.Motor(vex.Ports.PORT5, True)  # Reverse Polarity
huggyClaw  = vex.Motor(vex.Ports.PORT7) 
armLeft    = vex.Motor(vex.Ports.PORT8, True)  # Reverse Polarity
touchLED   = vex.Touchled(vex.Ports.PORT9) 
driveLeft  = vex.Motor(vex.Ports.PORT11) 
#endregion config

touchLED.on_rgb(255, 165, 0)

huggerOpened = False

drive = motor_group.MotorGroup((driveLeft, driveRight)) 
arm = motor_group.MotorGroup((armLeft, armRight))

def calibration():
    touchLED.on_rgb(0, 0, 255)
    gyro.set_heading()
    touchLED.on_rgb(0, 255, 0)
    while True:
        armLeft.spin(vex.DirectionType.REV, 70)
        armRight.spin(vex.DirectionType.REV, 70)
        if armLeft.current() > 0.5 and armRight.current() > 0.5:
            armLeft.stop()
            armRight.stop()
            armLeft.reset_rotation()
            armRight.reset_rotation()
            armLeft.set_max_torque_current(0.3)
            armRight.set_max_torque_current(0.3)
            break
    
    armLeft.spin_for(vex.DirectionType.FWD, 100, vex.RotationUnits.DEG, 100, vex.VelocityUnits.PCT, False)
    armRight.spin_for(vex.DirectionType.FWD, 100, vex.RotationUnits.DEG, 100, vex.VelocityUnits.PCT, True)

        
    armLeft.spin_for(vex.DirectionType.REV, 100, vex.RotationUnits.DEG, 100, vex.VelocityUnits.PCT, False)
    armRight.spin_for(vex.DirectionType.REV, 100, vex.RotationUnits.DEG, 100, vex.VelocityUnits.PCT, True)
    
    while True:
        huggyClaw.spin(vex.DirectionType.FWD, 40)
        if huggyClaw.current() > 0.12:
            huggyClaw.stop()
            huggyClaw.reset_rotation()
            huggyClaw.set_max_torque_current(0.2)
            moveHuggyClaw(True)
            print 'bruh'
            break
    
    return
        
def moveHuggyClaw(waitForEnd = False, wide = False):
    global huggerOpened
    print 'spinning'
    if not wide:
        if huggerOpened:
            huggyClaw.spin_to(-10, vex.RotationUnits.DEG, 100, vex.VelocityUnits.PCT, False)
        else:
            huggyClaw.spin_to(-170, vex.RotationUnits.DEG, 100, vex.VelocityUnits.PCT, False)
        
        if waitForEnd:
            vex.wait(0.2)
        
        huggerOpened = not huggerOpened
    else:
        huggyClaw.spin_to(-200, vex.RotationUnits.DEG, 100, vex.VelocityUnits.PCT, waitForEnd)

def driveStrafe(distance, velocity, direction, waitForEnd = True):
    print 'strafing'
    if direction == "Right":
        print 'right'
        driveH.spin_for(vex.DirectionType.FWD, distance, vex.RotationUnits.DEG, velocity, vex.VelocityUnits.PCT, waitForEnd)
        
    elif direction == "Left":
        driveH.spin_for(vex.DirectionType.REV, distance, vex.RotationUnits.DEG, velocity, vex.VelocityUnits.PCT, waitForEnd)

    return
            
def driveStrafeAlign(velocity, direction):
    if direction == "Right":            
        driveH.spin_for(vex.DirectionType.FWD, 100000, vex.RotationUnits.DEG, velocity, vex.TimeUnits.SEC, False)
        
    elif direction == "Left":
        driveH.spin_for(vex.DirectionType.REV, 100000, vex.RotationUnits.DEG, velocity, vex.TimeUnits.SEC, False)
    return

def moveArm(distance, velocity, direction, waitForEnd = True):
    """distance, velocity, direction, waitForEnd=True"""
    if direction == "Up":
        arm.spin_for(vex.DirectionType.FWD, distance, vex.RotationUnits.DEG, velocity, vex.VelocityUnits.PCT, waitForEnd)
        
    elif direction == "Down":
        arm.spin_for(vex.DirectionType.REV, distance, vex.RotationUnits.DEG, velocity,vex.VelocityUnits.PCT, waitForEnd)

    return

def armAlign(velocity, direction):
    if direction == "Up":            
        arm.spin_for(vex.DirectionType.FWD, 100000, vex.RotationUnits.DEG, velocity, vex.TimeUnits.SEC, False)
        
    elif direction == "Down":
        arm.spin_for(vex.DirectionType.REV, 100000, vex.RotationUnits.DEG, velocity, vex.TimeUnits.SEC, False)
    return

def driveStraight(distance, velocity, waitForEnd = True):
    drive.spin_for(vex.DirectionType.FWD, distance, vex.RotationUnits.DEG, velocity, vex.VelocityUnits.PCT, waitForEnd)

def throttleValues(value, throttler = 100):
    """
    Makes Sure no values are above the throttler amount
    This is helpful, because in RMS, if the velocity in 
    a spin() command is above 100, there will be unexp-
    ected results.
    """
    if value > throttler:
        return throttler
    elif value < -throttler:
        return -throttler
    else: return value
    
def correctGyroError(desiredValue, gyroValue):
    """
    Corrects the error in the Gyro sensor to make sure
    that the turns using the gyro sensor are always in
    the right direction.
    """
    tError = desiredValue - gyroValue
    if tError > 180:
        return tError - 360
    elif tError < -180:
        return tError + 360
    else:
        return tError

def driveStraightPID(desiredDistance, desiredHeading, maxVelocity = 90):
    """
    Drives straight using a PID loop for distance traveled,
    and a PID loop for gyro error correction.
    """
    maxError = 2
    maxDerivative = 0.5
    dP = 0.7
    dI = 0.0
    dD = 0.0
    
    tP = 0.7
    tI = 0.0
    tD = 0.3
    
    driveRight.reset_rotation()
    driveLeft.reset_rotation()
    
    tError = desiredHeading
    tPrevError = tError
    tDerivative = desiredHeading
    dError = desiredDistance
    dPrevError = dError
    dDerivative = desiredDistance
    
    # heading = gyro.heading()
    # tError = desiredHeading
    # dError = desiredDistance
    
    finished = False
    
    while not finished:
        if abs(tError) < maxError and abs(tDerivative) < maxDerivative:
            if desiredDistance - driveLeft.rotation() < 3:
                finished = True
        
        heading = gyro.heading()
        print round(driveLeft.rotation(), 2), round(driveRight.rotation(), 2), round(heading, 2), round(dError, 2), round(tError, 2)
        tError = correctGyroError(desiredHeading, heading)
        dError = desiredDistance - (driveLeft.rotation() + driveRight.rotation()) / 2
        
        dDerivative = (dError - dPrevError)
        tDerivative = (tError - tPrevError)
        
        dOutput = throttleValues((dError * dP + dDerivative * dD), 80)
        tOutput = (tError * tP + tDerivative * tD)
        
        driveLeft.spin(vex.DirectionType.FWD, throttleValues(dOutput - tOutput, maxVelocity))
        driveRight.spin(vex.DirectionType.FWD, throttleValues(dOutput + tOutput, maxVelocity))
        
        dPrevError = dError
        tPrevError = tError
        
        sys.sleep(0.010)
        
    driveLeft.stop()
    driveRight.stop()
    
def testArc(desiredHeading, ratio, direction, outerVel = 100):
    innerVel = outerVel * (ratio[1] / ratio[0])
    
    print outerVel, innerVel
    
    if direction == "Right":
        while True:
            error = gyro.heading() - desiredHeading
            print error
            if abs(error) < 3:
                break
            driveLeft.spin(vex.DirectionType.FWD, outerVel, vex.RotationUnits.DEG)
            driveRight.spin(vex.DirectionType.FWD, innerVel)
        
        driveLeft.stop(vex.BrakeType.HOLD)
        driveRight.stop(vex.BrakeType.HOLD)
        return
        
    elif direction == "Left":
        while True:
            error = gyro.heading() - desiredHeading
            print error
            if abs(error) < 3:
                break
            driveRight.spin(vex.DirectionType.FWD, outerVel, vex.RotationUnits.DEG)
            driveLeft.spin(vex.DirectionType.FWD, innerVel)
        driveRight.stop(vex.BrakeType.HOLD)
        driveLeft.stop(vex.BrakeType.HOLD)
    
    return

def turnGetDir(error):
    """
    Corrects the direction a turnPID turn will go,
    making it turn in the shortest traveled direc-
    tion
    """
    if error < 0:
        error += 360
    # print diff
    if error > 180:
        return "Left"
    else:
        return "Right"

def turnPID(desiredValue, holdingRiser = False):
    """
    Turns the robot using a PID loop with the 
    gyro sensor
    """
    # while True:
    #     driveRight.set_reversed(False)
    #     drive.spin(vex.DirectionType.REV, 102)
    steadyStateErrorCap = 1
    derivativeCap = 0.2
    if holdingRiser:
        kP = 1
        kI = 0.0
        kD = 0.3
        
    else:
        kP = 0.03 # 1.0  
        kI = 0
        kD = 0.02 # 0.9
    finished = False
    error = 0
    prevError = 0
    # print "starting move, aiming for", desiredHeading
    touchLED.on_rgb(0,0,255)
    while not finished:
        heading = gyro.heading()
        error = desiredValue - heading
        if error < 0:
            error += 360
        derivative = error - prevError
        prevError = error
        turnAmount = throttleValues((kP*error - kD*derivative))
        # turnAmount *= 0.7
        # drive.spin(vex.DirectionType.FWD, turnAmount)
        # drive.spin(vex.DirectionType.FWD, turnAmount)
        pidDir = turnGetDir(error)
        if pidDir == "Right": 
            driveRight.spin(vex.DirectionType.FWD, abs(turnAmount))
            driveLeft.spin(vex.DirectionType.REV, abs(turnAmount))
        else:
            driveRight.spin(vex.DirectionType.REV, abs(turnAmount))
            driveLeft.spin(vex.DirectionType.FWD, abs(turnAmount))
        print abs(turnAmount), pidDir
        # print abs(turnAmount)
        if abs(error) < steadyStateErrorCap and abs(derivative) < derivativeCap:
            gyro.set_heading(desiredValue)
            finished = True
        # vex.wait(0.05)
    drive.stop()
    driveRight.set_reversed(True)
    touchLED.on_rgb(0,255,0)
    print("finished move")
    
def run_part(function):
    driveRight.set_stopping(vex.BrakeType.HOLD)
    driveLeft.set_stopping(vex.BrakeType.HOLD)
    touchLED.on_rgb(0, 255, 0)
    function()
    
def rightSide1():
    testArc(260, (3.55, 1), "right", 100)
    sys.sleep(0.2)
    driveStraight(175, 80)
    moveHuggyClaw(True)
    
    armAlign(100, "Up")
    
    sys.sleep(0.4)
    
    turnPID(295, True)
    arm.stop()
    drive.set_timeout(1)
    driveStraight(-550, 60)
    drive.set_timeout(100)
    turnPID(270)
    
    driveStraight(540, 60, False)
    sys.sleep(0.8)
    # driveStrafe(200, 80, "Left", False)
    while True:
        if drive.is_done():
            break
    # driveStrafe(200, 60, "Left", False)
    # driveStraightPID(250, 270)
    
    armAlign(100, "Down")
    sys.sleep(0.4)
    arm.stop()
        
    moveHuggyClaw(False, True)
    arm.stop()
    
    driveStraight(-150, 80)
    
    if False:
    
        # driveStraight(-350, 80)
        # armAlign(100, "Up")
        # sys.sleep(1)
        # arm.stop()
        # driveStrafeAlign(70, "Right")
        # sys.sleep(2)
        # driveH.stop()
        # # driveStrafe(160, 80, "Left")
        
        # # driveStraightPID(420, 280)
        # # driveStrafe(150, 40, "Left", False)
        # driveStraight(250, 60)
        # # turnPID(285)
        # driveRight.spin_for(vex.DirectionType.FWD, 75, vex.RotationUnits.DEG, 75, vex.VelocityUnits.PCT, False)
        # driveLeft.spin_for(vex.DirectionType.REV, 75, vex.RotationUnits.DEG, 75, vex.VelocityUnits.PCT, False)
        
        # driveStrafe(100, 100, "Left")
        
        # drive.spin_for_time(vex.DirectionType.FWD, 0.9, vex.TimeUnits.SEC, 90)
        # driveStraight(-75, 60)
        
        # moveArm(150, 100, "Down")
        
        # moveHuggyClaw(False, True)
    
        # driveStraight(-150, 80)
        pass

def test1():
    pass
def rightSideTest():
    testArc(260, (5, 1), "Right", 60) 
    driveStrafeAlign(50, "Right")
    sys.sleep(1)
    driveH.stop()
    
    moveHuggyClaw()
    driveStraight(275, 40)
    
    driveStraight(-100, 50)
    
    driveStrafeAlign(50, "Right")
    sys.sleep(0.2)
    driveH.stop()
    
    moveHuggyClaw()
    
    driveStrafe(700, 50, "Left")
    
    driveStraight(200, 50)
    moveHuggyClaw(True)
    armAlign(100, "Up")
    sys.sleep(1.3)
    arm.stop(vex.BrakeType.HOLD)
    
    driveStraight(-60, 60)
    turnPID(240)
    driveStraight(150, 60)
    
    armAlign(100, "Down")
    sys.sleep(0.4)
    
    arm.stop()
    
    moveHuggyClaw(False, True)
    
    driveStraight(-100, 100)
    
autonSequence = "Calibrate"
autonSequenceAfterCali = "RightSideTest"
    
while True:
    if brain.buttonUp.pressing():
        armRight.spin(vex.DirectionType.FWD, 100, vex.VelocityUnits.PCT)
        armLeft.spin(vex.DirectionType.FWD, 100, vex.VelocityUnits.PCT)

    elif brain.buttonDown.pressing():
        armRight.spin(vex.DirectionType.REV, 100, vex.VelocityUnits.PCT)
        armLeft.spin(vex.DirectionType.REV, 100, vex.VelocityUnits.PCT)
    else:
        armRight.stop(vex.BrakeType.HOLD); armLeft.stop(vex.BrakeType.HOLD)
    touchLED.on_rgb(255, 0, 0)
    driveRight.set_stopping(vex.BrakeType.COAST)
    driveLeft.set_stopping(vex.BrakeType.COAST)
    if touchLED.pressing():
        if autonSequence == "Calibrate":
            run_part(calibration)
            autonSequence = autonSequenceAfterCali
            pass
        elif autonSequence == "RightSide1":
            run_part(rightSide1)
            autonSequence = "RightSide2"
            pass
        elif autonSequence == "RightSideTest":
            run_part(rightSideTest)
            autonSequence = "Calibrate"
            pass
        elif autonSequence == "RightSide2":
            touchLED.on_rgb(128, 128, 128)
            pass
        elif autonSequence == "Test1":
            run_part(test1)
            autonSequence = "RightSide2"
            pass
    
