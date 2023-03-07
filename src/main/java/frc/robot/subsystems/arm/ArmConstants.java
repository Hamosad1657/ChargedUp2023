
package frc.robot.subsystems.arm;

import com.hamosad1657.lib.math.HaUnits.PIDGains;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ArmConstants {
	public static final double kAngleMotorMaxSpeed = 0.3;
	public static final double kLengthMotorMaxOutput = 0.7;

	public static final double kAngleEncoderOffset = 352.8;
	public static final double kLengthEncoderOffset = 0.0;

	public static final PIDGains kArmAnglePIDGains = new PIDGains(0.02, 0.00001, 0.0);
	public static final double kArmAngleTolerance = 3.0;

	public static final PIDGains kArmLengthPIDGains = new PIDGains(0.0028, 0.0003, 0.0);
	public static final double kArmLengthTolerance = 150.0;

	public static final double kMaxAngleVelocityRadPS = 0.0;
	public static final double kMaxAngleAccelerationRadPSSquared = 0.0;
	public static final Constraints kAnglePIDConstrains = new TrapezoidProfile.Constraints(
			ArmConstants.kMaxAngleVelocityRadPS, ArmConstants.kMaxAngleAccelerationRadPSSquared);

	public static final ArmState[] kArmStates = { ArmState.kHome, ArmState.kLow, ArmState.kMid, ArmState.kHigh };

	public static enum ArmState {
		// Uses the relativity of the encoders as setpoints
		kHigh(83.0, 2100.0), kMid(0.0, 0.0), kLow(0.0, 0.0), kShelf(0.0, 0.0), kHome(0.0, 0.0);

		public final double angleDeg;
		public final double lengthDeg;

		ArmState(double angleDeg, double armLength) {
			this.angleDeg = angleDeg;
			this.lengthDeg = armLength;
		}
	}
}