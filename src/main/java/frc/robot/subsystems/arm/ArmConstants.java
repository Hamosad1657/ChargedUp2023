
package frc.robot.subsystems.arm;

import com.hamosad1657.lib.math.HaUnits.PIDGains;

public class ArmConstants {
	public static final double kArmAngleSpeedRatio = 0.5;
	public static final double kArmLengthSpeedRatio = 1.0;

	// TODO: Find the correct offset. It should measure 260 when the arm is at most closed position possible.
	public static final double kAngleCANCoderOffsetDeg = 352.8;
	// TODO: Find the correct offset. It should measure 10 when the arm is at the most retracted position possible.
	public static final double kArmLengthEncoderOffset = 0.0;

	// TODO: Tune the PID gains and tolerance.
	public static final double kArmAngleBalanceMotorOutput = 0.035;
	public static final PIDGains kArmAnglePIDGains = new PIDGains(0.001, 0.0, 0.0);
	public static final double kArmAngleTolerance = 0.0;
	public static final PIDGains kArmLengthPIDGains = new PIDGains(0.0, 0.0, 0.0);
	public static final double kArmLengthTolerance = 0.0;

	// TODO: Find FF gains
	public static final double kArmAngleFFkS = 0.0;
	public static final double kArmAngleFFkG = 0.0;
	public static final double kArmAngleFFkV = 0.0;

	public static final double kBottomAngleLimitDeg = 250.0;
	public static final double kTopAngleLimitDeg = 170.0;

	// Angle CANCoder
	public static final double kMinArmAngleDeg = 0;
	public static final double kMaxArmAngleDeg = 150;

	// TODO: Make sure the values are correct.
	// Length CANCoder
	public static final double kMinLength = 0;
	public static final double kMaxLength = 0;

	public static final double kArmLengthErrorPerDeg = 0.1; // TODO: Find the correct ratio.
	public static final double kMaxMotorOutput = 0.5; // TODO: Adjust according to the system's requirements.

	public static final double kMinArmLengthDeg = 0;
	public static final double kMaxArmLengthDeg = 150;

	public static enum ArmState {
		// Uses the relativity of the encoder as setpoints
		kHigh(0, 0), kMid(0, 0), kLowFront(0, 0), kLowBack(0, 0), kShelf(0, 0), kInsideRobot(0.0, 0.0);

		public static final ArmState kDefaultState = kInsideRobot;

		public final double angleDeg;
		public final double lengthDeg;

		ArmState(double angleDeg, double distanceMeters) {
			this.angleDeg = angleDeg;
			this.lengthDeg = distanceMeters;
		}
	}
}