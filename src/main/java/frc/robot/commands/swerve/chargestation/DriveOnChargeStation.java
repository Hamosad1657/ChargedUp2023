// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve.chargestation;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveOnChargeStation extends CommandBase {
	private SwerveSubsystem swerve;
	private IntakeSubsystem intake;
	private boolean isOnFloor = true;

	public DriveOnChargeStation(SwerveSubsystem swerve, IntakeSubsystem intake) {
		this.swerve = swerve;
		this.intake = intake;
		this.addRequirements(this.swerve, this.intake);
	}

	private boolean isAngleInTolerance(double angle) {
		return angle < BalanceChassisConstants.kGroundAngle + BalanceChassisConstants.kTolerance
				&& angle > BalanceChassisConstants.kGroundAngle - BalanceChassisConstants.kTolerance;
	}

	private boolean isBalanced() {
		double roll = this.swerve.getRoll().getDegrees();
		return this.isAngleInTolerance(roll);
	}

	@Override
	public void initialize() {
		// this.intake.setIntakeMotor(-IntakeConstants.kDeafultSpeed);
	}

	@Override
	public void execute() {
		// if (!this.isBalanced()) {
		// //this.intake.setIntakeMotor(0.0);
		// }

		// if (this.isOnFloor) {
		// this.swerve.autonomousDrive(new ChassisSpeeds(BalanceChassisConstants.kFloorDriveSpeedMPS, 0, 0), false,
		// true);
		// } else {
		// this.swerve.autonomousDrive(new ChassisSpeeds(BalanceChassisConstants.kChargeStationDriveSpeedMPS, 0, 0),
		// false, true);
		// }
		this.swerve.autonomousDrive(new ChassisSpeeds(BalanceChassisConstants.kChargeStationDriveSpeedMPS, 0, 0), false,
				true);
	}

	@Override
	public void end(boolean interrupted) {
		this.intake.raiseIntakeCommand();
		this.swerve.crossLockWheels();
		Robot.print("Ended");
	}

	@Override
	public boolean isFinished() {
		return this.isBalanced();
	}
}
