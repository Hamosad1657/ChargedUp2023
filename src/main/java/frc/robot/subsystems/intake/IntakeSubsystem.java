
package frc.robot.subsystems.intake;

import com.hamosad1657.lib.motors.HaTalonSRX;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class IntakeSubsystem extends SubsystemBase {
	private static IntakeSubsystem instance;

	public static IntakeSubsystem getInstance() {
		if (instance == null) {
			instance = new IntakeSubsystem();
		}
		return instance;
	}

	private final ShuffleboardTab intakeTab;
	private final HaTalonSRX intakeMotor;
	private final DigitalInput raiseLimit, lowerLimit;
	private boolean isIntakeOpen;

	private IntakeSubsystem() {
		this.intakeTab = Shuffleboard.getTab("Intake");

		this.intakeMotor = new HaTalonSRX(RobotMap.kIntakeMotorID);
		this.intakeMotor.setIdleMode(IdleMode.kBrake);
		this.isIntakeOpen = false;
		this.raiseLimit = new DigitalInput(RobotMap.kIntakeRaiseLimitPort);
		this.lowerLimit = new DigitalInput(RobotMap.kIntakeLowerLimitPort);

		this.intakeTab.add("Raise Limit", this.raiseLimit);
		this.intakeTab.add("Lower Limit", this.lowerLimit);
	}

	public Command lowerIntakeCommand() {
		return new StartEndCommand(() -> this.intakeMotor.set(-IntakeConstants.kMotorDefaultOutput), () -> {
			this.intakeMotor.set(0.0);
			this.isIntakeOpen = true;
		}, this).until(() -> !this.lowerLimit.get()).withTimeout(IntakeConstants.kCommandTimoutSec);
	}

	public Command raiseIntakeCommand() {
		return new StartEndCommand(() -> this.intakeMotor.set(IntakeConstants.kMotorDefaultOutput), () -> {
			this.intakeMotor.set(0.0);
			this.isIntakeOpen = false;
		}, this).until(() -> !this.raiseLimit.get()).withTimeout(IntakeConstants.kCommandTimoutSec);
	}

	public Command keepIntakeUpCommand() {
		return new InstantCommand(
				() -> this.intakeMotor.set(this.isIntakeOpen ? 0.0 : IntakeConstants.kKeepInPlaceOutput), this);
	}
}
