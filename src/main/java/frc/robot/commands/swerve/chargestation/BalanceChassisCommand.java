
package frc.robot.commands.swerve.chargestation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class BalanceChassisCommand extends CommandBase {
	private SwerveSubsystem swerve;
	private PIDController pid;

	public BalanceChassisCommand(SwerveSubsystem swerve) {
		this.pid = BalanceChassisConstants.kPIDGains.toPIDController();
		this.pid.setSetpoint(BalanceChassisConstants.kGroundAngle);
		this.pid.setTolerance(BalanceChassisConstants.kTolerance);

		this.swerve = swerve;
		this.addRequirements(this.swerve);
	}

	@Override
	public void initialize() {
		Robot.print("Started Auto Balance Command");
	}

	@Override
	public void execute() {
		double vxMeters = MathUtil.clamp(this.pid.calculate(this.swerve.getRoll().getDegrees()),
				-BalanceChassisConstants.kDriveSpeedMPS, BalanceChassisConstants.kDriveSpeedMPS);

		this.swerve.autonomousDrive(new ChassisSpeeds(vxMeters, 0, 0), false, true);
	}

	@Override
	public void end(boolean interrupted) {
		Robot.print("Finished Balance Chassis Command");
	}

	@Override
	public boolean isFinished() {
		return this.pid.atSetpoint();
	}
}