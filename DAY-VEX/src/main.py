# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Christian McBride & Michael Day                              #
# 	Created:      5/4/2026, 2:15:01 PM                                         #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

from vex import *

brain = Brain()

rightMotor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False) # Right drivetrain motor
leftMotor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True) # Left drivetrain motor
liftMotor = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False) # Lift motor
inertial_1 = Inertial(Ports.PORT5) # Inertial sensor
liftArmLocation = Rotation(Ports.PORT6) # Lift arm location sensor
bumperSwitch = Bumper(brain.three_wire_port.a) # Bumper switch

def bump():
    """
    Hold the program's execution until the bumper switch is pressed
    """
    while not bumperSwitch.pressing():
        wait(10, MSEC)

        brain.screen.set_cursor(1,1)
        brain.screen.print("Press the button to start the program")
        pass

    brain.screen.clear_line(1)
    brain.screen.set_cursor(1,1)
    brain.screen.print("Program executed")
    wait(1, SECONDS)

def inertialCalibration():
    """
    1. Calibrate the inertial sensor
    2. Include a 2 second wait time for calibration
    3. Call this function at the start of the program's execution
    """

    brain.screen.clear_screen()
    brain.screen.set_cursor(1,1)
    brain.screen.print("Calibrating the initerial sensor")
    brain.screen.set_cursor(2,1)
    brain.screen.print("Don't move the robot")
    inertial_1.calibrate()          # Calibrate the intertial sesor

    wait(2, SECONDS)                # Time required to calibrate the inertial sensor

    brain.screen.set_cursor(1,1)
    brain.screen.clear_line()
    brain.screen.print("Intertial calibration complete")


def driveStraightData(e):
    """
    Report position, rotation, and error data to the brain's screen
    Parameter: e = calculated error value
    """

    brain.screen.set_cursor(1,1)
    brain.screen.print("Position: " + str(leftMotor.position())) # Print the left motor's position value
    brain.screen.set_cursor(2,1)
    brain.screen.print("Rotation: " + str(inertial_1.rotation())) # Print the inertial sensor's rotation value
    brain.screen.set_cursor(3,1)
    brain.screen.print("Error: " + str(e)) # Print the error value

def stopMotors():
    """
    Stop both drivetrain motors at the same time
    """

    rightMotor.stop()
    leftMotor.stop()
    wait(0.5, SECONDS) # Wait for 0.5 seconds for the system to stabilize

def driveStraight(distance, setPoint, motorVelocity):
    """
    Drive the robot straight for a specified distance
    Parameters:
        distance: The distance to travel (in inches)
        setPoint: equal to zero degrees for driving straight
        motorVelocity: The nominal velocity of the motors (+) => forward, (-) => backward
    """

    inertial_1.reset_rotation() # Reset the inertial sensor's rotation to zero

    leftMotor.set_stopping(COAST)
    rightMotor.set_stopping(COAST)

    kP = 0.6 # Proportional gain for driving straight
             # Used to calculate the correction to mainain course
             # If to small, correction will occur to slowly
             # If too large, overcorrection will occur
             # Determine best value iteratively through testing

    wheelDiameter = 4 # 4 inch wheel diameter
    wheelCircumference = wheelDiameter * math.pi # Calculate wheel circumference

    #Convert the distance in inches to distance in ticks
    # distance (ticks) = (distance (inches) / wheel circumference) * ticks per revolution
    distanceTicks = (distance / wheelCircumference) * 360

    # Reset the motor encoders
    leftMotor.set_position(0, DEGREES)
    rightMotor.set_position(0, DEGREES)

    # Drive forward if motor velocity is positive, backward if negative
    if(motorVelocity > 0):
        while leftMotor.position(DEGREES) < distanceTicks:
            e = setPoint - inertial_1.rotation() # Calculate error
            correction = kP * e # Calculate motor velocity correction

            # Correct motor velocities
            # If e > 0, (setpoint > rotation) => robot is veering to the left
            # If e < 0, (setpoint < rotation) => robot is veering to the right
            leftMotor.set_velocity(motorVelocity + correction, PERCENT)
            rightMotor.set_velocity(motorVelocity - correction, PERCENT)

            # Spin the motors
            leftMotor.spin(FORWARD)
            rightMotor.spin(FORWARD)

            driveStraightData(e) # Report data to the brain's screen
        stopMotors() # Stop the motors once the target distance is reached
    else:
        distanceTicks *= -1
        while leftMotor.position(DEGREES) > distanceTicks:
            e = setPoint - inertial_1.rotation() # Calculate error
            correction = kP * e # Calculate motor velocity correction

            # Correct motor velocities
            # If e > 0, (setpoint > rotation) => robot is veering to the left
            # If e < 0, (setpoint < rotation) => robot is veering to the right
            leftMotor.set_velocity(motorVelocity + correction, PERCENT)
            rightMotor.set_velocity(motorVelocity - correction, PERCENT)

            # Spin the motors
            leftMotor.spin(FORWARD)
            rightMotor.spin(FORWARD)

            driveStraightData(e) # Report data to the brain's screen
        stopMotors() # Stop the motors once the target distance is reached

def turnData(turnError, derivative):
    brain.screen.set_cursor(1,1)
    brain.screen.print("Heading: " + str(inertial_1.heading())) # Print the inertial sensor's heading
    brain.screen.set_cursor(2,1)
    brain.screen.print("Error: " + str(abs(turnError)))         # Print the turnError
    brain.screen.set_cursor(3,1)
    brain.screen.print("Derivative: " + str(abs(derivative)))   # Print the derivative value

def pointTurn(setPoint):
    """
    1. Perform a point turn using the interial sensor heading and proportional & derivative control
    2. Argument: Desired heading (setPoint)
    """

    brain.screen.clear_screen()
    # Set the stopping mode for the left and right motors
    leftMotor.set_stopping(BRAKE)
    rightMotor.set_stopping(BRAKE)

    # PID constants
    kP_CW = 0.05
    kD_CW = 0.02
    # Use direct variables for CW and CCW kP and kD
    kP_CCW = 0.05
    kD_CCW = 0.02

    # Define maximum turning velocity and the previous error term
    maxVelocity = 50    # Maximum turning velocity
    previousError = 0   # Error from previous iteration of the loop

    while (True):
        rawError = setPoint - inertial_1.heading()      # Calculates raw error
        turnError = (rawError + 180) % 360 - 180        # Wrap turn error between -180 and 180
        derivative = turnError - previousError           # Calculates derivative term from previous error

        # Determines CW or CCW based off of error sign of turnError
        if turnError >= 0:
            kP = kP_CW
            kD = kD_CW
        else:
            kP = kP_CCW
            kD = kD_CCW

        turnData(turnError, derivative)    # Heading, error, and derivative data printed

        # Break out of the loop and stop turning when the setPoint is reached without oscillation
        if ((abs(turnError) < 1 ) and (abs(derivative) < 0.2)):
            stopMotors()            # Stop the motors
            break                   # Exit the while loop

        # Calculate the correction for the motor velocities
        turnCorrection = (kP * turnError) + (kD * derivative)

        # Limit the turnCorrection value to be between -1 and 1
        # Will keep the motor velocity less than or equal to the maxVelocity
        if (turnCorrection > 1):
            turnCorrection = 1
        elif (turnCorrection < -1):
            turnCorrection = -1

        turnVelocity = maxVelocity * turnCorrection

        # Set motor velocities. We don't need a clockwise if statement
        # If turnVelocity is positive, Left goes forward, Right goes back -> CW
        # If turnVelocity is negative, Left goes back, Right goes forward -> CCW
        leftMotor.set_velocity(turnVelocity)
        rightMotor.set_velocity(-1 * turnVelocity)

        # Spin the motors
        leftMotor.spin(FORWARD)
        rightMotor.spin(FORWARD)

        previousError = turnError  # Updates the previousError term
        wait(20, MSEC)

def main():
    """
    The main() function is the program that is executed by the brain
    """

    bump()
    leftMotor.set_stopping(BRAKE)
    rightMotor.set_stopping(BRAKE)
    inertialCalibration() # Calibrate the inertial sensor

    # driveStraight(87, 0, 50)
    # wait(4, SECONDS) # Wait for 4 seconds
    # driveStraight(87, 0, -50)

    pointTurn(224)
    wait(2, SECONDS)
    pointTurn(37)
    wait(2, SECONDS)
    pointTurn(135)
    wait(2, SECONDS)


main()