
package frc.robot.commands.swerve.autonomous.balanceChassis;

import com.hamosad1657.lib.vision.limelight.Limelight;
import com.hamosad1657.lib.vision.limelight.LimelightConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.commands.swerve.autonomous.balanceChassis.BalanceChassisConstants.BalancingOptions;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class BalanceChassisCommand extends CommandBase {
	private SwerveSubsystem chassis;
	private PIDController pid;
	private BalancingOptions balancingOption;
	private Alliance alliance;

	public BalanceChassisCommand(SwerveSubsystem chassis, BalancingOptions balancingOption) {
		this.pid = BalanceChassisConstants.kPIDGains.toPIDController();
		this.pid.setSetpoint(BalanceChassisConstants.kGroundAngle);
		this.pid.setTolerance(BalanceChassisConstants.kTolerance);

		this.balancingOption = balancingOption;

		this.chassis = chassis;
		this.addRequirements(this.chassis);
	}

	@Override
	public void initialize() {
		if (DriverStation.getAlliance() == Alliance.Blue) {
			this.alliance = Alliance.Blue;
		} else {
			this.alliance = Alliance.Red;
		}
		Robot.print("Started Auto Balance Command");
	}

	@Override
	public void execute() {
		switch (this.balancingOption) {
		case kBangBangLimelight:
			if (this.alliance == Alliance.Blue) {
				this.bangBangBalance(
						Limelight.getBotpose_wpiBlue(LimelightConstants.kLimelightName).getRotation().getY());
			} else {
				this.bangBangBalance(
						Limelight.getBotpose_wpiRed(LimelightConstants.kLimelightName).getRotation().getY());
			}
			break;

		case kBangBangNavX:
			this.bangBangBalance(this.chassis.getRoll().getDegrees());
			break;

		case kPIDLimelight:
			if (this.alliance == Alliance.Blue) {
				this.pidBalance(Limelight.getBotpose_wpiBlue(LimelightConstants.kLimelightName).getRotation().getY());
			} else {
				this.pidBalance(Limelight.getBotpose_wpiRed(LimelightConstants.kLimelightName).getRotation().getY());
			}
			break;

		case kPIDNavX:
			this.pidBalance(this.chassis.getRoll().getDegrees());
			break;
		}
	}

	@Override
	public void end(boolean interrupted) {
		this.chassis.crossLockWheels();
		Robot.print("Finished Balance Chassis Command");
	}

	@Override
	public boolean isFinished() {
		switch (this.balancingOption) {
		case kBangBangLimelight:
			if (this.alliance == Alliance.Blue) {
				return this.isFinishedBangBang(Limelight.getBotpose_wpiBlue(LimelightConstants.kLimelightName).getY());
			} else {
				return this.isFinishedBangBang(Limelight.getBotpose_wpiRed(LimelightConstants.kLimelightName).getY());
			}

		case kBangBangNavX:
			return this.isFinishedBangBang(this.chassis.getRoll().getDegrees());

		case kPIDLimelight:
		case kPIDNavX:
			return this.isFinishedPID();

		default:
			return false;
		}
	}

	private void bangBangBalance(double currentAngle) {
		// Move forwards if the angle is smaller and backwards if it's larger
		double direction = 0.0;
		if (currentAngle < BalanceChassisConstants.kGroundAngle + BalanceChassisConstants.kTolerance) {
			direction = 1.0;
		} else if (currentAngle > BalanceChassisConstants.kGroundAngle - BalanceChassisConstants.kTolerance) {
			direction = -1.0;
		}

		this.chassis.autonomousDrive(new ChassisSpeeds(BalanceChassisConstants.kDriveSpeedMPS * direction, 0, 0), true,
				true);
	}

	private boolean isFinishedBangBang(double currentAngle) {
		return currentAngle <= BalanceChassisConstants.kGroundAngle + BalanceChassisConstants.kTolerance
				&& currentAngle >= BalanceChassisConstants.kGroundAngle - BalanceChassisConstants.kTolerance;
	}

	private void pidBalance(double currentAngle) {
		double vxMeters = MathUtil.clamp(this.pid.calculate(currentAngle), -2.0, 2.0);
		this.chassis.autonomousDrive(new ChassisSpeeds(vxMeters, 0, 0), false, true);
	}

	private boolean isFinishedPID() {
		return this.pid.atSetpoint();
	}
}
