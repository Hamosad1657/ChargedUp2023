
package frc.robot.subsystems.intake;

import com.hamosad1657.lib.math.HaUnits.PIDGains;

public class IntakeConstants {
	public static final double kAngleMotorDefaultOutput = 0.625;
	public static final double kAngleMotorKeepInPlaceOutput = 0.075;

	public static final double kAngleCANCoderOffset = -77;

	public static final PIDGains kAngleMotorGains = new PIDGains(0.04, 0.001, 0.001);
	public static final double kAngleMotorMaxPIDOutput = 1.0;
	public static final double kAngleTolerance = 1.0;
	public static final double kCollectWithCubeMaxVelocity = 50.0;

	public static final double kShootDuration = 0.5;
	public static final double kShootCollectDuration = 0.1;

	public static final double kIntakeMotorCollectOutput = -0.4;

	public enum ShootHeight {
		kMid(126.5, 0.155), kHigh(125.0, 0.275), kLow(75.0, 0.125), kFar(90.0, 1.0), kAuto(90.0, 0.33);

		public final double motorOutput;
		public final double angle;

		private ShootHeight(double angle, double motorOutput) {
			this.angle = angle;
			this.motorOutput = motorOutput;
		}
	}

	public static final double kInitialPIDBoostMaxAngle = 130.0;
	public static final double kInitialPIDBoostMinAngle = 100.0;
	public static final double kKeepIntakeUpOutput = 0.03;
	public static final double kIntitailPIDBoost = 0.35;
}
