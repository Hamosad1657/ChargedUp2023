
package frc.robot.subsystems.arm;

import com.hamosad1657.lib.math.HaUnits.PIDGains;

public class ArmConstants {
	public static final double kArmAngleSpeedRatio = 0.3;
	public static final double kArmLengthSpeedRatio = 0.6;

	// TODO: Find the correct offset. It should measure 260 when the arm is at most closed position possible.
	public static final double kAngleCANCoderOffsetDeg = 352.8;
	// TODO: Find the correct offset. It should measure 10 when the arm is at the most retracted position possible.
	public static final double kArmLengthEncoderOffset = 0.0;

	public static final double kArmAngleBalanceMinDeg = 13.0;
	public static final double kArmAngleBalanceMinMotorOutput = 0.0225;
	public static final double kArmAngleBalanceMiddleDeg = 26.0;
	public static final double kArmAngleBalanceMiddleMotorOutput = 0.035;

	public static final double kArmAngleBottomThreshold = 18.0;
	public static final double kArmAngleTopThreshold = 74.0;
	public static final double kArmAngleThresholdSpeedRatio = 0.5;

	public static final double kArmLengthRetractThreshold = 800.0;
	public static final double kArmLengthExtendThreshold = 2500.0;
	public static final double kArmLengthThresholdSpeedRatio = 0.5;

	// TODO: Tune the PID gains and tolerance.
	public static final PIDGains kArmAnglePIDGains = new PIDGains(0.02, 0.00001, 0.0);
	public static final double kArmAngleTolerance = 0.0;
	public static final PIDGains kArmLengthPIDGains = new PIDGains(0.0028, 0.0003, 0.0);
	public static final double kArmLengthTolerance = 0.0;

	// TODO: Find FF gains
	public static final double kArmAngleFFkS = 0.0;
	public static final double kArmAngleFFkG = 0.0;
	public static final double kArmAngleFFkV = 0.0;

	// TODO: verify
	public static final double kBottomAngleLimitDeg = 250.0;
	public static final double kTopAngleLimitDeg = 170.0;

	// TODO: check all limits and input them in correctly
	// Angle CANCoder

	// Length CANCoder
	public static final double kMinArmLengthDeg = 0;
	public static final double kMaxArmLengthDeg = 0;

	public static final double kMaxMotorOutput = 0.5; // TODO: Adjust according to the system's requirements.

	public static enum ArmState {
		// Uses the relativity of the encoders as setpoints
		kHigh(0, 0), kMid(60, 2200), kLowFront(0, 0), kLowBack(0, 0), kShelf(0, 0), kInsideRobot(0.0, 0.0);

		public static final ArmState kDefaultState = kInsideRobot;

		public final double angleDeg;
		public final double lengthDeg;

		ArmState(double angleDeg, double distanceMeters) {
			this.angleDeg = angleDeg;
			this.lengthDeg = distanceMeters;
		}
	}
}