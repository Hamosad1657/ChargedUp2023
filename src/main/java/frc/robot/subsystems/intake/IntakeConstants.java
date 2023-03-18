
package frc.robot.subsystems.intake;

import com.hamosad1657.lib.math.HaUnits.PIDGains;

public class IntakeConstants {
	public static final double kAngleMotorDefaultOutput = 0.5;
	public static final double kAngleMotorKeepRaisedOutput = 0.1;

	public static final double kAngleCANCoderOffset = 0.0;

	public static final PIDGains kIntakeMotorGains = new PIDGains(0.02, 0.0, 0.01);
	public static final double kAngleTolerance = 1.0;
	public static final double kShootAngleSetpoint = 105.0;

	public static final double kShootDuration = 0.75;
	public static final double kShootCollectDuration = 0.1;
	public static final double kIntakeMotorCollectOutput = -0.4;

	public enum ShootHeight {
		kMid(0.75), kHigh(1.0), kAuto(1.0);

		public final double motorOutput;

		private ShootHeight(double motorOutput) {
			this.motorOutput = motorOutput;
		}
	}
}
