# Path Planner Instructions
This is the instruction file for the path makers.
It includes the all the avilable commands and common combinitions of them.

# Command list
### NOTE: ALWAYS PUT THE EVENTS IN SQUENTAL MODE.
- **ToggleGrabber** - Toggles the grabber.
- **OpenIntake** - Opens the intake.
- **CloseIntake** - Closes the intake.
### NOTE: The arm commands dont work ATM, do not use
- **InsideRobot** - Gets the arm inside the robot.
- **ArmHigh** - Gets the arm to the third level.
- **ArmMiddle** - Gets the arm to the second level.
- **ArmLow** - Gets the arm to the low level.

### NOTE: EVERY PATH ENDS WITH CROSS LOCKING OF THE SWERVE WHEELS

# Common combinations:
- **Charge Station** - When creating a path from 0 it is recomended to always end the path with the charge station, aswell as making the path go out of the community first (Please make the path to the community from the side that isnt the one with the wire protection, since it will most likly ruin the odometry).
 
# One piece:
- **Arm** - If the game piece is inside the grabber, you need give it the wanted arm state ( low, mid, high) and then toggle grabber, after that it is recommended that you use the "inside robot" command before getting on the charge station.
- **Intake** - It is recommended that you put the cone / cube on the intake itself and then open it and it will go right into the low spot, after that make sure to close the intake.
# The Goal: Two game pieces
- **Intake** - As of this time we are unable to collect a second game peice with the arm, and so we need to collect the second game piece with the intake and take it with us to a bottom slot, after that we need to create a straight path where we close the intake and drive straight moving the game piece with us into the bottom slot.

### NOTE: Reminding that as of writing this, the arm do not work.

