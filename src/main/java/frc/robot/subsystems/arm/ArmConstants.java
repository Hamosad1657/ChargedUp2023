
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

	public static final double kArmLengthRetractThreshold = 300.0;
	public static final double kArmLengthExtendThreshold = 2200.0;
	public static final double kArmLengthThresholdSpeedRatio = 0.65;

	// TODO: Tune the PID gains and tolerance.
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
	public static final double kArmAngleTolarenceDeg = 0;

	// TODO: Make sure the values are correct.
	// Length CANCoder
	public static final double kMinLength = 0.714 * ArmConstants.kArmLengthToPulleyDistanceRatio;
	public static final double kMaxLength = 1.3877 * ArmConstants.kArmLengthToPulleyDistanceRatio;

	public static final double kArmLengthTolarenceDeg = 0;

	public static final double kArmLengthToPulleyDistanceRatio = 0.1; // TODO: Find the correct ratio.
	public static final double kMaxMotorOutput = 0.5; // TODO: Adjust according to the system's requirements.

	public static final double kMinArmLengthDeg = 0;
	public static final double kMaxArmLengthDeg = 150;

	public static enum ArmState {
		kHigh(96, 1.2 * ArmConstants.kArmLengthToPulleyDistanceRatio),
		kMid(80, 0.725 * ArmConstants.kArmLengthToPulleyDistanceRatio),
		kLowFront(30.5, 1.177 * ArmConstants.kArmLengthToPulleyDistanceRatio),
		kLowBack(30, 1.1 * ArmConstants.kArmLengthToPulleyDistanceRatio),
		kShelf(80, 0.700 * ArmConstants.kArmLengthToPulleyDistanceRatio), kInsideRobot(30, kMinLength); // Lenght is in
																										// meters

		public static final ArmState kDefaultState = kInsideRobot;

		public final double angleDeg;
		public final double lengthMeters;

		ArmState(double angleDeg, double distanceMeters) {
			this.angleDeg = angleDeg;
			this.lengthMeters = distanceMeters;
		}

		public double getPulleyDistance() {
			return this.lengthMeters * ArmConstants.kArmLengthToPulleyDistanceRatio;
		}
	}
}