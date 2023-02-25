
package frc.robot.commands.swerve.autonomous.balanceChassis;

import com.hamosad1657.lib.math.HaUnits.PIDGains;

public class BalanceChassisConstants {
	public static final double kGroundAngle = 0;
	public static final double kTolerance = 1.5;
	public static final double kDriveSpeed = 0.5;
	public static final PIDGains kPIDGains = new PIDGains(0.05, 0.0, 0.0);

	public enum BalancingOptions {
		kBangBangLimelight, kBangBangNavX, kPIDLimelight, kPIDNavX
	}

}
