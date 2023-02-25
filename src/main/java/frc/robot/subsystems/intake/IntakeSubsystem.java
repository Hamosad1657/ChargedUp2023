
package frc.robot.subsystems.intake;

import com.hamosad1657.lib.motors.HaTalonSRX;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {
	private static IntakeSubsystem instance;

	public static IntakeSubsystem getInstance() {
		if (instance == null) {
			instance = new IntakeSubsystem();
		}
		return instance;
	}

	private HaTalonSRX intakeMotor;
	private boolean isIntakeOpen;

	private IntakeSubsystem() {
		this.intakeMotor = new HaTalonSRX(RobotMap.kIntakeMotorID);
		this.intakeMotor.setIdleMode(IdleMode.kBrake);
		isIntakeOpen = false;
	}

	/**
	 * Toggles the intake's motors.
	 */
	public Command toggleIntake() {
		if (isIntakeOpen) {
			this.isIntakeOpen = false;
			return new SequentialCommandGroup(
					new InstantCommand(() -> this.intakeMotor.set(IntakeConstants.kDeafultSpeed), this),
					new WaitCommand(IntakeConstants.kWaitingTime),
					new InstantCommand(() -> this.intakeMotor.set(0), this));
		}
		this.isIntakeOpen = true;
		return new SequentialCommandGroup(
				new InstantCommand(() -> this.intakeMotor.set(-IntakeConstants.kDeafultSpeed), this),
				new WaitCommand(IntakeConstants.kWaitingTime), new InstantCommand(() -> this.intakeMotor.set(0), this));
	}

	public Command lowerIntakeCommand() {
		this.isIntakeOpen = false;
		return new SequentialCommandGroup(
				new InstantCommand(() -> this.intakeMotor.set(IntakeConstants.kDeafultSpeed), this),
				new WaitCommand(IntakeConstants.kWaitingTime), new InstantCommand(() -> this.intakeMotor.set(0), this));
	}

	public Command raiseIntakeCommand() {
		this.isIntakeOpen = true;
		return new SequentialCommandGroup(
				new InstantCommand(() -> this.intakeMotor.set(-IntakeConstants.kDeafultSpeed), this),
				new WaitCommand(IntakeConstants.kWaitingTime), new InstantCommand(() -> this.intakeMotor.set(0), this));
	}

	public void setIntakeMotor(double speedPercentOutput) {
		this.intakeMotor.set(speedPercentOutput);
	}
}
