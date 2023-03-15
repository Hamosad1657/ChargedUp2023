
package frc.robot.subsystems.turret;

import com.hamosad1657.lib.math.HaUnits.PIDGains;

public class TurretConstants {
	public static final double kMotorMaxOutput = 0.8;
	public static final double kCANCoderOffsetDeg = 15.3;

	public static final double kRotationMinAngle = 32.0;
	public static final double kRotationMaxAngle = 350.0;

	public static final PIDGains kRotationPIDGains = new PIDGains(0.03, 0.0, 0.0002);
	public static final double kRotationTolerance = 2.0;
	public static final double kAutoRotationTolerance = 20.0;
	public static final double kRotationTeleopSetpointMultiplier = 1.4;

	public static final double kFrontRotationSetpoint = 270.0;
	public static final double kBackRotationSetpoint = 90.0;
}