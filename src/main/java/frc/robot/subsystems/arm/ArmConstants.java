
package frc.robot.subsystems.arm;

import com.hamosad1657.lib.math.HaUnits.PIDGains;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;;

public class ArmConstants {
	public static final double kAngleMotorMaxOutput = 0.6;
	public static final double kLengthMotorMaxOutput = 1.0;

	public static final double kAngleEncoderOffset = 0.0;
	public static final double kLengthEncoderOffset = 0.0;

	public static final double kHomingAngleOutput = -0.2;
	public static final double kHomingLengthOutput = 1.0;
	public static final double kHomingLengthKeepRetractedOutput = 0.125;
	public static final double kHomingAnglePIDRatio = 1.45;
	/** For homing - The max arm's length that still counts as retracted. */
	public static final double kHomingRetractedMaxLength = 300.0;
	/** For homing - The lowest angle the arm can go to when it's extended. */
	public static final double kHomingExtendedMinAngle = 80.0;

	public static final PIDGains kAnglePIDGains = new PIDGains(0.02725, 0.0075, 0.0);
	public static final double kAngleTolerance = 1.0;
	public static final double kAngleMotorMaxPIDOutput = 0.75;
	public static final double kAngleDownOutputRatio = 0.5;

	public static final double kLengthRetractMinAngle = 50.0;

	public static final double kAngleMaxVelocityDegPS = 140.0;
	public static final double kAngleMaxAccelerationDegPS = 200.0;
	public static final Constraints kAnglePIDConstrains = new Constraints(kAngleMaxVelocityDegPS,
			kAngleMaxAccelerationDegPS);

	public static final double kAngleTeleopSetpointMultiplier = 1.25;
	public static final double kAngleMaxSetpoint = 119.0;
	public static final double kAngleMinSetpoint = 35.0;

	public static final PIDGains kArmLengthPIDGains = new PIDGains(0.0017, 0.0000325, 0.0);
	public static final double kLengthTolerance = 50.0;

	public static enum ArmState {
		// Uses the relativity of the encoders as setpoints
		kHigh(119.0, 3000.0), kMid(96.0, 1100.0), kLowCone(54.5, 1200.0), kLowConePickup(52.5, 2200.0),
		kLowRaiseCone(50.0, 2300.0), kLowConeDropoff(65.0, 1000.0), kLowCube(47.0, 2100.0), kShelf(107.0, 0.0),
		kHalfClosed(48.0, 400.0);

		public final double angleDeg;
		public final double lengthDeg;

		ArmState(double angleDeg, double lengthDeg) {
			this.angleDeg = angleDeg;
			this.lengthDeg = lengthDeg;
		}
	}
}