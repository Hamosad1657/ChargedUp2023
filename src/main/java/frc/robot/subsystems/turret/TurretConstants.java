
package frc.robot.subsystems.turret;

import com.hamosad1657.lib.math.HaUnits.PIDGains;

public class TurretConstants {
	public static final int kCCWRotationLimitSwitchPort = 1;
	public static final int kCWRotationLimitSwitchPort = 0;

	public static final PIDGains kRotationPIDGains = new PIDGains(0.1, 0.0, 0.0);
	public static final double kFrontAngleSetpoint = 0.0;
	public static final double kSpeedRatio = 0.25;

	public static final double kCANCoderOffsetDeg = -106.3;
	public static final double kCANCoderCCWLimitDeg = -176.0; // Counter Clock Wise
	public static final double kCANCoderCWLimitDeg = 176.0; // Clock Wise

	// TODO find actual angles
	public static final double kAngleAtCCWLimit = 0.0;
	public static final double kAngleAtCWLimit = 0.0;
}