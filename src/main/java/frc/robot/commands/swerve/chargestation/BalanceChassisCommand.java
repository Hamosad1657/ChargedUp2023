
package frc.robot.commands.swerve.chargestation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class BalanceChassisCommand extends CommandBase {
	private SwerveSubsystem swerve;
	private PIDController balanceController;
	private Timer balanceTimer;

	public BalanceChassisCommand(SwerveSubsystem swerve) {
		this.balanceController = BalanceChassisConstants.kPIDGains.toPIDController();
		this.balanceController.setSetpoint(BalanceChassisConstants.kGroundAngle);
		this.balanceController.setTolerance(BalanceChassisConstants.kTolerance);
		this.balanceTimer = new Timer();

		this.swerve = swerve;
		this.addRequirements(this.swerve);
	}

	@Override
	public void initialize() {
		Robot.print("Started Auto Balance Command");
		this.balanceTimer.start();
	}

	@Override
	public void execute() {
		double vyMeters = MathUtil.clamp(this.balanceController.calculate(this.swerve.getPitch().getDegrees()),
				-BalanceChassisConstants.kDriveSpeedMPS, BalanceChassisConstants.kDriveSpeedMPS);

		this.swerve.autonomousDrive(new ChassisSpeeds(0, vyMeters, 0), false, true);

		if (!this.balanceController.atSetpoint()) {
			this.balanceTimer.reset();
		}
	}

	@Override
	public void end(boolean interrupted) {
		Robot.print("Finished Balance Chassis Command");
	}

	@Override
	public boolean isFinished() {
		return this.balanceTimer.hasElapsed(BalanceChassisConstants.kWaitingTimeSec);
	}
}