# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Christian McBride & Michael Day                              #
# 	Created:      5/4/2026, 2:15:01 PM                                         #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

brain.screen.print("Hello V5")

#-----------------------------Robot Configuration-----------------------------#
rightMotor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False) # Right drivetrain motor 
leftMotor = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)   # Left drivetrain motor 
liftMotor = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)  # Lift motor 
inertial_1 = Inertial(Ports.PORT5)                             # Inertial Sensor 
liftArmRotation = Rotation(Ports.PORT6, False)                 # Liftarm rotation sensor
bumpSwitch = Bumper(brain.three_wire_port.a)                   # Bumper Switch 
#-----------------------------------------------------------------------------# 

#-----------------------------Helper functions---------------------------------#
def bump():
    """
    Hold the program's execution until the button is pressed. 
    """

    while(bumpSwitch.pressing() == False):
        wait(10, MSEC)  # Debounce the button  

        brain.screen.set_cursor(1,1)     # Place the cursor in row 1, col. 1
        brain.screen.print("Press the button to start the program")
        pass
    brain.screen.clear_line(1)
    brain.screen.set_cursor(1,1)
    brain.screen.print("Program executed")
    wait(1, SECONDS)


def inertialCalibration():
    """
    1. Calibrate the inertial sensor. 
    2. Include a 2 second wait time for calibration. 
    3. Call this function at the start of the program's execution 
    """

    brain.screen.clear_screen()
    brain.screen.set_cursor(1,1)
    brain.screen.print("Calibrating the inertial sensor")
    brain.screen.set_cursor(2,1)
    brain.screen.print("Don't move the robot!")
    inertial_1.calibrate() #  Calibrate the inertial sensor 

    wait(2, SECONDS)       #  Time required to calibrate the inertial sensor 

    brain.screen.set_cursor(1,1)
    brain.screen.clear_line()
    brain.screen.print("Inertial calibration complete")


def testInertial(): 
    """
    1. Test the inertial sensor by having it display heading and rotation data 
    2. Press the button to end the test 
    """

    brain.screen.clear_screen()
    while(bumpSwitch.pressing() == False):
        wait(10, MSEC)  #Debounce the button 
        brain.screen.set_cursor(5,1)
        brain.screen.print("Heading:  " + str(inertial_1.heading())) 
        brain.screen.set_cursor(6,1)
        brain.screen.print("Rotation:  " + str(inertial_1.rotation())) 
        brain.screen.set_cursor(8,1)
        brain.screen.print("Press the button to end the test")


#------------------------------Define main() function------------------------------#
def main():
    """
    The main() function is the program that is executed by the Brain 
    """

    bump()                     # Call the bump() function to begin program execution 
    inertialCalibration()      # Calibrate the inertial sencor 
    testInertial()             # Test the inertial sensor 

main()