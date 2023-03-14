
package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;

/** This class contains ports and CAN IDs. */
public class RobotMap {
	public static int kDriverA_ControllerUSBPort = 0;
	public static int kDriverB_ControllerUSBPort = 1;
	public static SerialPort.Port kNavXPort = SerialPort.Port.kUSB1;

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

	public static final int kTurretCANCoderID = 14;
	public static final int kIntakeMotorID = 19;
	public static final int kArmLengthMotorID = 20;
	public static final int kArmLengthCANCoderID = 22;
	public static final int kArmAngleCANCoderID = 23;
	public static final int kTurretMotorID = 24;
	public static final int kArmAngleMotorID = 25;
	public static final int kGrabberMotorID = 26;

	// Turret limit switches
	public static final int kTurretCWLimitPort = 0;
	public static final int kTurretCCWLimitPort = 1;

	// Arm limit switches
	public static final int kArmExtendLimitPort = 2;
	public static final int kArmRetractLimitPort = 3;
	public static final int kBottomArmAngleLimitport = 4;
	public static final int kTopArmAngleLimitport = 5;

	// Intake limit switches
	public static final int kIntakeRaiseLimitPort = 6;
	public static final int kIntakeLowerLimitPort = 7;
}
