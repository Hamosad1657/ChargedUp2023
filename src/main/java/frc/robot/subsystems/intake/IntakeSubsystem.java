
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

	private final HaTalonSRX intakeMotor;
	private boolean isIntakeOpen;

	private IntakeSubsystem() {
		this.intakeMotor = new HaTalonSRX(RobotMap.kIntakeMotorID);
		this.intakeMotor.setIdleMode(IdleMode.kBrake);
		this.isIntakeOpen = false;
	}

	public Command lowerIntakeCommand() {
		return new SequentialCommandGroup(
				new InstantCommand(() -> this.intakeMotor.set(-IntakeConstants.kMotorDefaultOutput), this),
				new WaitCommand(IntakeConstants.kLoweringWaitingTime), new InstantCommand(() -> {
					this.intakeMotor.set(0.0);
					this.isIntakeOpen = true;
				}, this));

	}

	public Command raiseIntakeCommand() {
		return new SequentialCommandGroup(
				new InstantCommand(() -> this.intakeMotor.set(IntakeConstants.kMotorDefaultOutput), this),
				new WaitCommand(IntakeConstants.kRaisingWaitingTime), new InstantCommand(() -> {
					this.intakeMotor.set(0.0);
					this.isIntakeOpen = false;
				}, this));
	}

	public Command keepIntakeUpCommand() {
		return new InstantCommand(
				() -> this.intakeMotor.set(this.isIntakeOpen ? 0.0 : IntakeConstants.kKeepInPlaceOutput), this);
	}

	public void setIntakeMotor(double speedPercentOutput) {
		this.intakeMotor.set(speedPercentOutput);
	}
}
