
package frc.robot.subsystems.intake;

import com.hamosad1657.lib.math.HaUnits.PIDGains;

public class IntakeConstants {
	public static final double kAngleMotorDefaultOutput = 0.625;
	public static final double kAngleMotorKeepInPlaceOutput = 0.075;

	public static final double kAngleCANCoderOffset = 0.0;

	public static final PIDGains kIntakeMotorGains = new PIDGains(0.005, 0.0, 0.000);
	public static final double kBalanceFFOutput = 0.01;
	public static final double kAngleMotorMaxPIDOutput = 0.85;
	public static final double kAngleTolerance = 1.0;
	public static final double kCollectWithCubeMaxVelocity = 50.0;

	public static final double kShootDuration = 0.5;
	public static final double kShootCollectDuration = 0.1;

	public static final double kIntakeMotorCollectOutput = -0.4;

	public enum ShootHeight {
		kMid(126.5, 0.2), kHigh(125.0, 0.275), kLow(75.0, 0.125), kFar(90.0, 1.0), kAuto(105.0, 1.0);

		public final double motorOutput;
		public final double angle;

		private ShootHeight(double angle, double motorOutput) {
			this.angle = angle;
			this.motorOutput = motorOutput;
		}
	}
}
