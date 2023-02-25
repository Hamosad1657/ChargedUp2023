
package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.hamosad1657.lib.math.HaUnits.PIDGains;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.fusionLib.swerve.SwerveModuleConstants;
import frc.robot.RobotMap;

public final class SwerveConstants {
	public static final boolean invertGyro = false; // HaNavX already inverts the navX
	public static final double kSwerveTranslateRatioFast = 0.85, kSwerveSpinRatioFast = 0.7;
	public static final double kSwerveTranslateRatioSlow = 0.3, kSwerveSpinRatioSlow = 0.3;

	/* Drivetrain Constants */
	public static final double kTrackWidthM = 0.42;
	public static final double kWheelBaseM = 0.42;
	public static double kWheelDiameterM = 0.1016;
	public static final double kWheelCircumferenceM = kWheelDiameterM * Math.PI;

	/*
	 * Swerve Kinematics No need to ever change this unless you are not doing a traditional rectangular/square 4 module
	 * swerve
	 */
	public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
			new Translation2d(kWheelBaseM / 2.0, kTrackWidthM / 2.0),
			new Translation2d(kWheelBaseM / 2.0, -kTrackWidthM / 2.0),
			new Translation2d(-kWheelBaseM / 2.0, kTrackWidthM / 2.0),
			new Translation2d(-kWheelBaseM / 2.0, -kTrackWidthM / 2.0));

	// Swerve pose estimator
	public static final Matrix<N3, N1> kVisionMeasurementStdDevs = new Matrix<N3, N1>(VecBuilder.fill(0.1, 0.1, 0.1));
	public static final Matrix<N3, N1> kStateStdDevs = new Matrix<N3, N1>(VecBuilder.fill(0.1, 0.1, 0.1));

	// Gear ratios
	public static final double kDriveGearRatio = 6.75 / 1.0;
	public static final double kSteerGearRatio = 12.8 / 1.0;

	// Motor inverts
	public static final boolean kDriveMotorInvert = false;
	public static final boolean kSteerMotorInvert = false;

	// Encoder invert
	public static final boolean kCANCoderInvert = false;

	// Steer current limits
	public static final int kSteerContinuousCurrentLimit = 25;
	public static final int kSteerPeakCurrentLimit = 35;
	public static final double kSteerPeakCurrentTimeSec = 0.1;
	public static final boolean kSteerEnableCurrentLimit = true;
	// Drive current limits
	public static final int kDriveContinuousCurrentLimit = 35;
	public static final int kDrivePeakCurrentLimit = 40;
	public static final double kDrivePeakCurrentTimeSec = 0.1;
	public static final boolean kDriveEnableCurrentLimit = true;

	/*
	 * These values are used by the drive falcon to ramp in open loop and closed loop driving. We found a small open
	 * loop ramp (0.25) helps with tread wear, tipping, etc
	 */
	public static final double kOpenLoopRampRate = 0.25;
	public static final double kClosedLoopRampRate = 0.0;

	// Steer motor PID values
	public static final double kSteerP = 0.2;
	public static final double kSteerI = 0.0;
	public static final double kSteerD = 0.0;
	public static final double kSteerFF = 0.0;

	// Drive motor PID values
	public static final double kDriveP = 0.05;
	public static final double kDriveI = 0.0;
	public static final double kDriveD = 0.02;
	public static final double kDriveFF = 0.0;

	// TODO: tune robot angle PID
	/*
	 * Used so that the error is in radians and the output is in radPS. So for example, if kP is 0.5, and you have an
	 * error of 1 radian, it will output 0.5 radians per second.
	 */

	// Robot angle PID values
	public static final PIDGains kRobotAnglePIDGains = new PIDGains(5, 0.0, 0.0);
	public static final double kRobotAngleToleranceRad = 0.0; // 2.0 * Math.PI / 360.0; // 1 Degree tolerance
	public static final double kRobotAngleFF = 0.0;

	/*
	 * Drive Motor Characterization Values Divide SYSID values by 12 to convert from volts to percent output for CTRE
	 */
	public static final double driveKS = (0.32 / 12); // TODO: This must be tuned to specific robot
	public static final double driveKV = (1.51 / 12);
	public static final double driveKA = (0.27 / 12);

	/* Swerve Profiling Values */
	/** Meters per Second */
	public static final double kChassisMaxSpeedMPS = 5;
	/** Radians per Second */
	public static final double kMaxAngularVelocityRadPS = 11.9;

	/* Neutral Modes */
	public static final NeutralMode kSteerNeutralMode = NeutralMode.Brake;
	public static final NeutralMode kDriveNeutralMode = NeutralMode.Brake;

	/* Module Specific Constants */

	/*
	 * To find the offsets set them to zero, deploy, turn the wheels so they all point forwards with the bevel gears
	 * facing to the left, then take the angle displayed in the "swerve" tab in the Shuffleboard and set them as the
	 * offsets.
	 */

	/* Front Left Module - Module 0 */
	public static final class FrontLeftModule {
		public static final int kDriveMotorID = RobotMap.kFrontLeftDriveMotorID;
		public static final int kSteerMotorID = RobotMap.kFrontLeftSteerMotorID;
		public static final int kCANCoderID = RobotMap.kFrontLeftCANCoderID;
		public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(257.34);
		public static final SwerveModuleConstants constants = new SwerveModuleConstants(kDriveMotorID, kSteerMotorID,
				kCANCoderID, kAngleOffset);

		public static final double kCrossAngleDeg = 45.0;
	}

	/* Front Right Module - Module 1 */
	public static final class FrontRightModule {
		public static final int kDriveMotorID = RobotMap.kFrontRightDriveMotorID;
		public static final int kSteerMotorID = RobotMap.kFrontRightSteerMotorID;
		public static final int kCANCoderID = RobotMap.kFrontRightCANCoderID;
		public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(130.16);
		public static final SwerveModuleConstants constants = new SwerveModuleConstants(kDriveMotorID, kSteerMotorID,
				kCANCoderID, kAngleOffset);
		public static final double kCrossAngleDeg = -45.0;
	}

	/* Back Left Module - Module 2 */
	public static final class BackLeftModule {
		public static final int kDriveMotorID = RobotMap.kBackLeftDriveMotorID;
		public static final int kSteerMotorID = RobotMap.KBackLeftSteerMotorID;
		public static final int kCANCoderID = RobotMap.kBackLeftCANCoderID;
		public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(359.29);
		public static final SwerveModuleConstants constants = new SwerveModuleConstants(kDriveMotorID, kSteerMotorID,
				kCANCoderID, kAngleOffset);
		public static final double kCrossAngleDeg = 135.0;
	}

	/* Back Right Module - Module 3 */
	public static final class BackRightModule {
		public static final int kDriveMotorID = RobotMap.kBackRightDriveMotorID;
		public static final int kSteerMotorID = RobotMap.kBackRightSteerMotorID;
		public static final int canCoderID = RobotMap.kBackRightCANCoderID;
		public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(91.4);
		public static final SwerveModuleConstants constants = new SwerveModuleConstants(kDriveMotorID, kSteerMotorID,
				canCoderID, kAngleOffset);
		public static final double kCrossAngleDeg = -135.0;
	}
}