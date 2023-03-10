
package frc.robot.subsystems.turret;

import com.hamosad1657.lib.math.HaUnits.PIDGains;

public class TurretConstants {
	public static final double kMotorMaxOutput = 0.65;
	public static final double kCANCoderOffsetDeg = 13.0;

	public static final double kRotationMinAngle = 32.0;
	public static final double kRotationMaxAngle = 350.0;

	public static final PIDGains kRotationPIDGains = new PIDGains(0.065, 0.0, 0.0);
	public static final double kRotationTolerance = 1.0;
	public static final double kRotationTeleopSetpointMultiplier = 1.25;

	public static final double kFrontRotationSetpoint = 90.0;
	public static final double kBackRotationSetpoint = 270.0;
}