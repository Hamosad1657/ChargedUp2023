# Driving Instructions
This is the instruction file for the drivers.
It includes the autonomous behavior of the robot and the manual control / overrides available.

<br>

---

<br>

## Driver A (Samech):

*(Talk with Shaked or Aviv if you wnat to change anything)*
### **Swerve**
### Autonomous
- **None** (at the moment).

### Teleop
- **Left stick's Y-axis:** Move robot forwards or backwards.
- **Left stick's X-axis:** Move robot left or rightr.
- **Right stick's X-axis:** Rotate robot left or right.
- **Share button:** Zero gyro
- **Cross button:** Cross-lock wheels
- **Triangle button:** Toggle swerve speed
- **PS button:** Wheels forward (to prepare to start moving, getting out of X is slightly slower)

<br>

### **Intake**
### Autonomous
- **None** (at the moment).

### Teleop
- **R1** - Raise intake.
- **L1** - Lower intake.

---

<br>

## Driver B (Hamburger / Maayan):

*(Talk with Berg if you want to change anything)*

<br>

### **Grabber:**
### Autonomous
- **None** (at the moment).

### Teleop
- **Circle button:** Toggles the grabber when pressed.

<br>

### **Arm**
### Autonomous (pre-determind states)
- Keyboard input:
	- **1-3:** Low front (for picking up game-pieces or placing them in the bottom row of nodes).
	- **4-6:** Mid front (for placing game-pieces in the middle row of nodes).
	- **7-9:** High front (for placing game-pieces in the top row of nodes).
- Controller input:
	- **Triangle button:** Shelf (for picking up game-pieces from the shelf).
	- **Cross button:** Inside robot (for holding a game-piece inside of the robot or for the start and end of the match).
	- **Square button:** High (for putting game pieces in the top level of the gird).
	- **R1** - Low front (for putting game pieces in the low level of the gird).
	- **L1** - Middle (for putting game pieces in the middle level of the gird). (might change for low back in the future if the shelf state is good for the middle state aswell.)

	- **Touchpad** - Get to the chosen state.

### Teleop
- **R2 trigger**: Opens (extends) the arm, faster when pressed harder.
- **L2 trigger**: Closes (retracts) the arm, faster when pressed harder.
- **Right stick's Y-axis:** Controls the angle of the arm.

<br>

### **Turret**
### Autonomous (pre-determind states)
- **D-Pad Up** Sets the turret position to the front of the chassis.
- **D-Pad Down** Sets the turret position to the back of the chassis.
- **D-Pad Left** Switches the D-Pad down location to the left back.
- **D-Pad Right** witches the D-Pad down location to the right back.

### Teleop
- **Left stick's X-axis:** Rotates the turret left or right.

<br>

### **Instructions to find swerve offsets:**
- Set all offsets in file SwerveConstants.java to zero.
- Deploy robot code, wait until it says "build succsessful".
- Power off robot, and physically turn the wheels to point forwards (bevel gears facing to the left.)
- Ensure that wheels are perfectly straight using a profile.
- Power on robot and connect to it's network with WiFi/USB/Ethernet.
- Go to "swerve" tab in the Shuffleboard (it opens when you start the Driver Station).
- Take the measured angles and copy them into the offsets in SwerveConstants.java file.
- Deploy robot code. Done!