
package frc.robot.subsystems.arm;

import com.hamosad1657.lib.math.HaUnits.PIDGains;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ArmConstants {
	public static final double kAngleMotorMaxSpeed = 0.3;
	public static final double kLengthMotorMaxOutput = 1.0;

	public static final double kAngleEncoderOffset = 352.8;
	public static final double kLengthEncoderOffset = 0.0;

	public static final PIDGains kArmAnglePIDGains = new PIDGains(0.02, 0.00001, 0.0);
	public static final double kArmAngleTolerance = 3.0;

	public static final PIDGains kArmLengthPIDGains = new PIDGains(0.0028, 0.0003, 0.0);
	public static final double kArmLengthTolerance = 150.0;

	public static final double kMaxAngleVelocityRadPS = Math.PI / 4.0;
	public static final double kMaxAngleAccelerationRadPSSquared = kMaxAngleVelocityRadPS / 2.0;

	public static enum ArmState {
		// Uses the relativity of the encoders as setpoints
		kHigh(80.0, 245.0), kMid(62.0, 817.0), kLowCone(10.0, 1498.0), kLowCube(0.0, 0.0), kShelf(74.0, 397.0);

		public final double angleDeg;
		public final double lengthDeg;

		ArmState(double angleDeg, double armLength) {
			this.angleDeg = angleDeg;
			this.lengthDeg = armLength;
		}
	}
}