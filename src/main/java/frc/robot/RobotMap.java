
package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;

/** This class contains CAN IDs and port numbers. */
public class RobotMap {
	public static int kDriverAControllerUSBPort = 0;
	public static int kDriverBControllerUSBPort = 1;
	public static SerialPort.Port kNavXPort = SerialPort.Port.kUSB1;
	public static final I2C.Port kColorSensorPort = I2C.Port.kMXP;

	// Front left module
	public static final int kFrontLeftDriveMotorID = 15;
	public static final int kFrontLeftSteerMotorID = 5;
	public static final int kFrontLeftCANCoderID = 6;

	// Front right module
	public static final int kFrontRightDriveMotorID = 16;
	public static final int kFrontRightSteerMotorID = 7;
	public static final int kFrontRightCANCoderID = 8;

	// Back left module
	public static final int kBackLeftDriveMotorID = 17;
	public static final int KBackLeftSteerMotorID = 9;
	public static final int kBackLeftCANCoderID = 10;

	// Back right module
	public static final int kBackRightDriveMotorID = 18;
	public static final int kBackRightSteerMotorID = 11;
	public static final int kBackRightCANCoderID = 12;

	public static final int kIntakeMotorID = 19;

	// These are in the same order as the physical wiring
	public static final int kArmLengthMotorAID = 20;
	public static final int kArmLengthMotorBID = 21;
	public static final int kArmLengthCANCoderID = 22;
	public static final int kArmAngleCANCoderID = 23;
	public static final int kArmAngleMotorID = 25;
	public static final int kTurretCANCoderID = 14;
	public static final int kTurretMotorID = 24;
	// TODO: Get the real motor ID
	public static final int kGrabberMotorID = 26;

	// Turret limit switches
	public static final int kTurretCWLimitPort = 0;
	public static final int kTurretCCWLimitPort = 1;

	// Arm limit switches
	public static final int kArmRetractLimitPort = 2;
	public static final int kArmExtendLimitPort = 3;
	public static final int kBottomArmAngleLimitport = 4;
	public static final int kTopArmAngleLimitport = 5;
}
