
package frc.robot.subsystems.intake;

import com.hamosad1657.lib.math.HaUnits.PIDGains;

public class IntakeConstants {
	public static final double kAngleMotorDefaultOutput = 0.8;
	public static final double kAngleMotorKeepRaisedOutput = 0.1;

	public static final double kAngleCANCoderOffset = 0.0;

	public static final PIDGains kIntakeMotorGains = new PIDGains(0.0, 0.0, 0.0);
	public static final double kAngleTolerance = 0.1;

	public static final double kIntakeCollectMotorOutput = -0.5;
	public static final double kShootAngleSetpoint = 0.0;
	public static final double kShootDuration = 0.5;

	public enum ShootHeight {
		kMid(0.325), kHigh(0.75), kAuto(1.0);

		public final double motorOutput;

		private ShootHeight(double motorOutput) {
			this.motorOutput = motorOutput;
		}
	}
}
