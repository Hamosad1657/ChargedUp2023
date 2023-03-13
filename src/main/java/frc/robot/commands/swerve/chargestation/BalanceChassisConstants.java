
package frc.robot.commands.swerve.chargestation;

import com.hamosad1657.lib.math.HaUnits.PIDGains;

public class BalanceChassisConstants {
	public static final double kGroundAngle = 0.0;
	public static final double kTolerance = 2.5;
	public static final double kDriveSpeedMPS = 0.7;
	public static final PIDGains kPIDGains = new PIDGains(0.06, 0, 0.025);
}