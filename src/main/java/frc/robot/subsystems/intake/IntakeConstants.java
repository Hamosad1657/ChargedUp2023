
package frc.robot.subsystems.intake;

import com.hamosad1657.lib.math.HaUnits.PIDGains;

public class IntakeConstants {
	public static final double kAngleMotorDefaultOutput = 0.625;
	public static final double kAngleMotorKeepInPlaceOutput = 0.075;

	public static final double kAngleCANCoderOffset = -77;

	public static final PIDGains kAngleMotorGains = new PIDGains(0.006, 0.0006, 0.0005);
	public static final double kAngleMotorMaxPIDOutput = 1.0;
	public static final double kAngleTolerance = 1.0;
	public static final double kCollectWithCubeMaxVelocity = 50.0;

	public static final double kShootDuration = 0.5;
	public static final double kShootCollectDuration = 0.1;

	public static final double kIntakeMotorCollectOutput = -0.4;

	public static final double kInitialPIDBoostMaxAngle = 176.0;
	public static final double kInitialPIDBoostMinAngle = 100.0;
	public static final double kKeepIntakeUpOutput = 0.0;
	public static final double kIntitailPIDBoost = 0.25;
	public static final double kAngleMotorPositiveCompensation = 0.1;

	public enum ShootHeight {
		kMid(167, 0.155), kHigh(163.70, 0.3), kLow(90, 0.25), kFar(130.0, 1.0), kAuto(130.0, 0.33);

		public final double motorOutput;
		public final double angle;

		private ShootHeight(double angle, double motorOutput) {
			this.angle = angle;
			this.motorOutput = motorOutput;
		}
	}
}
