
package frc.robot.subsystems.arm;

import com.hamosad1657.lib.math.HaUnits.PIDGains;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ArmConstants {
	public static final double kArmAngleSpeedRatio = 0.3;
	public static final double kArmLengthSpeedRatio = -0.6;

	public static final double kAngleCANCoderOffsetDeg = 352.8;
	public static final double kArmLengthEncoderOffset = 0.0;

	public static final double kArmAngleBalanceMinDeg = 13.0;
	public static final double kArmAngleBalanceMinMotorOutput = 0.0225;
	public static final double kArmAngleBalanceMiddleDeg = 26.0;
	public static final double kArmAngleBalanceMiddleMotorOutput = 0.035;

	public static final double kArmAngleBottomThreshold = 16.0;
	public static final double kArmAngleTopThreshold = 76.0;
	public static final double kArmAngleThresholdSpeedRatio = 0.5;

	public static final double kArmLengthRetractThreshold = 200.0;
	public static final double kArmLengthExtendThreshold = 5000.0;
	public static final double kArmLengthThresholdSpeedRatio = 0.75;

	public static final double kAngleP = 0.02;
	public static final double kAngleI = 0.00001;
	public static final double kAngleD = 0.0;
	public static final double kMaxAngleVelocityRadPerSecond = 0.0;
	public static final double kMaxAngleAccelerationRadPerSecSquared = 0.0;
	public static final double kAngleControllerUpdateWaitPeriod = 0.0;
	public static final Constraints kAngleTrapezoidProfile = new TrapezoidProfile.Constraints(
			ArmConstants.kMaxAngleVelocityRadPerSecond, ArmConstants.kMaxAngleAccelerationRadPerSecSquared);

	public static final double kArmAngleTolerance = 3.0;
	public static final PIDGains kArmLengthPIDGains = new PIDGains(0.0028, 0.0003, 0.0);
	public static final double kArmLengthTolerance = 150.0;

	// TODO: verify
	public static final double kBottomAngleLimitDeg = 36.5;
	public static final double kTopAngleLimitDeg = 120.0;
	public static final double kMinArmLengthDeg = 0;
	public static final double kMaxArmLengthDeg = 0;

	public static final double kMaxLengthMotorOutput = 0.7;

	public static double maxLengthForAngle(double angle) {
		// return (-0.434085 * angle * angle) + (96.3868 * angle) - 250.936;
		if (angle < 24.0) {
			return 250.0;
		}

		if (angle < 28.0) {
			return 1800.0;
		}

		return 2600.0;
	}

	public static enum ArmState {
		// Uses the relativity of the encoders as setpoints
		kHigh(83.0, 2100.0);

		public final double angleDeg;
		public final double lengthDeg;

		ArmState(double angleDeg, double armLength) {
			this.angleDeg = angleDeg;
			this.lengthDeg = armLength;
		}
	}
}