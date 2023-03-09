
package frc.robot.subsystems.arm;

import com.hamosad1657.lib.math.HaUnits.PIDGains;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;;

public class ArmConstants {
	public static final double kAngleMotorMaxOutput = 0.3;
	public static final double kLengthMotorMaxOutput = 1.0;

	public static final double kAngleEncoderOffset = 0.0;
	public static final double kLengthEncoderOffset = 0.0;

	public static final double kHomingAngleOutput = -0.175;
	public static final double kHomingLengthOutput = 1.0;
	public static final double kHomingDownWaitTime = 0.25;
	/** For homing - The max arm's length that still counts as retracted. */
	public static final double kHomingRetractedMaxLength = 300.0;
	/** For homing - The lowest angle the arm can go to when it's extended. */
	public static final double kHomingExtendedMinAngle = 80.0;

	public static final PIDGains kAnglePIDGains = new PIDGains(0.018, 0.0, 0.0);
	public static final double kAngleTolerance = 1.5;
	public static final double kAngleMotorMaxPIDOutput = 0.5;
	public static final double kAngleDownOutputRatio = 0.5;
	public static final double kLengthExtendMinAngle = 40.0;

	public static final double kAngleMaxVelocityDegPS = 120.0;
	public static final double kAngleMaxAccelerationDegPS = 200.0;
	public static final Constraints kAnglePIDConstrains = new Constraints(kAngleMaxVelocityDegPS,
			kAngleMaxAccelerationDegPS);

	public static final double kAngleTeleopSetpointMultiplier = 1.25;
	public static final double kAngleMaxSetpoint = 118.0;
	public static final double kAngleMinSetpoint = 35.0;

	public static final PIDGains kArmLengthPIDGains = new PIDGains(0.001, 0.0, 0.0);
	public static final double kLengthTolerance = 50.0;

	public static enum ArmState {
		// Uses the relativity of the encoders as setpoints
		kHigh(117.0, 2800.0), kMid(94.0, 1000.0), kLowCone(52.0, 1100.0), kLowConePickup(52.0, 1700.0),
		kLowRaiseCone(48.0, 2000.0), kLowCube(41.5, 1700.0), kShelf(106.0, 100.0);

		public final double angleDeg;
		public final double lengthDeg;

		ArmState(double angleDeg, double armLength) {
			this.angleDeg = angleDeg;
			this.lengthDeg = armLength;
		}
	}
}