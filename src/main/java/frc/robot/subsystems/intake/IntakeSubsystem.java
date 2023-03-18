
package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.hamosad1657.lib.sensors.HaCANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.intake.IntakeConstants.ShootHeight;

public class IntakeSubsystem extends SubsystemBase {
	private static IntakeSubsystem instance;

	public static IntakeSubsystem getInstance() {
		if (instance == null) {
			instance = new IntakeSubsystem();
		}
		return instance;
	}

	/** Negative output lowers the intake, positive output raises the intake. */
	private final CANSparkMax angleMotor;
	private final PIDController angleController;
	private final HaCANCoder angleCANCoder;
	private final WPI_TalonFX intakeMotor;
	private final DigitalInput raiseLimit, lowerLimit;

	private boolean isIntakeLowered;

	private IntakeSubsystem() {
		this.angleMotor = new CANSparkMax(RobotMap.kIntakeAngleMotorID, MotorType.kBrushless);
		this.angleMotor.setIdleMode(IdleMode.kCoast);

		this.angleController = IntakeConstants.kIntakeMotorGains.toPIDController();
		this.angleController.setTolerance(IntakeConstants.kAngleTolerance);

		this.angleCANCoder = new HaCANCoder(RobotMap.kIntakeAngleCANCoderID, IntakeConstants.kAngleCANCoderOffset);
		this.angleCANCoder.setMeasurmentRange(AbsoluteSensorRange.Unsigned_0_to_360);

		this.intakeMotor = new WPI_TalonFX(RobotMap.kIntakeMotorID);
		this.intakeMotor.setNeutralMode(NeutralMode.Brake);
		this.intakeMotor.setInverted(true);

		// The limits are wired normally true, false when pressed
		this.raiseLimit = new DigitalInput(RobotMap.kIntakeRaiseLimitPort);
		this.lowerLimit = new DigitalInput(RobotMap.kIntakeLowerLimitPort);

		this.isIntakeLowered = false;

		if (Robot.showShuffleboardSubsystemInfo) {
			ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");

			intakeTab.add("Raise Limit", this.raiseLimit).withPosition(0, 0).withSize(1, 1);
			intakeTab.add("Lower Limit", this.lowerLimit).withPosition(1, 0).withSize(1, 1);

			intakeTab.addDouble("Intake Angle", this.angleCANCoder::getAbsAngleDeg).withPosition(2, 0).withSize(1, 1);
			intakeTab.addDouble("Intake Setpoint", this.angleController::getSetpoint).withPosition(3, 0).withSize(1, 1);
			intakeTab.addDouble("Intake Angle Error", this.angleController::getPositionError).withPosition(0, 1)
					.withSize(3, 3).withWidget(BuiltInWidgets.kGraph);
			intakeTab.addBoolean("Intake Angle At Setpoint", this.angleController::atSetpoint).withPosition(3, 1)
					.withSize(2, 1);

			intakeTab.addDouble("Intake Motor Output", this.intakeMotor::get).withPosition(3, 2).withSize(2, 1);
		}
	}

	public Command lowerIntakeCommand() {
		return new StartEndCommand(() -> {
			this.angleMotor.set(-IntakeConstants.kAngleMotorDefaultOutput);
		}, () -> {
			this.angleMotor.set(0.0);
			this.isIntakeLowered = true;
		}, this).until(() -> !this.lowerLimit.get());
	}

	public Command raiseIntakeCommand() {
		return new StartEndCommand(() -> {
			this.angleMotor.set(IntakeConstants.kAngleMotorDefaultOutput);
		}, () -> {
			this.angleMotor.set(0.0);
			this.isIntakeLowered = false;
		}, this).until(() -> !this.raiseLimit.get());
	}

	public Command keepRaisedCommand() {
		return new InstantCommand(
				() -> {
					this.angleMotor.set(this.isIntakeLowered ? 0.0 : IntakeConstants.kAngleMotorKeepRaisedOutput);
				},
				this);
	}

	public Command getToShootAngleCommand() {
		return new FunctionalCommand(() -> {
			this.angleController.setSetpoint(IntakeConstants.kShootAngleSetpoint);
		}, () -> {
			this.angleMotor.set(this.angleController.calculate(this.angleCANCoder.getAbsAngleDeg()));
		}, (interrupted) -> {
			this.angleMotor.set(0.0);
		}, () -> {
			return this.angleController.atSetpoint();
		}, this);
	}

	public Command shootCommand(ShootHeight height) {
		return this.getToShootAngleCommand()
				.deadlineWith(new InstantCommand(() -> this.intakeMotor.set(IntakeConstants.kIntakeCollectMotorOutput)))
				.andThen(new StartEndCommand(() -> {
					this.intakeMotor.set(height.motorOutput);
				}, () -> {
					this.intakeMotor.set(0.0);
				}, this).withTimeout(IntakeConstants.kShootDuration));
	}

	public Command autoCollectPieceCommand() {
		return new StartEndCommand(() -> {
			this.intakeMotor.set(IntakeConstants.kIntakeCollectMotorOutput);
		}, () -> {
			this.intakeMotor.set(0.0);
		}, this).withTimeout(IntakeConstants.kShootDuration);
	}

	public Command autoReleasePieceCommand() {
		return new StartEndCommand(() -> {
			this.intakeMotor.set(-ShootHeight.kAuto.motorOutput);
		}, () -> {
			this.intakeMotor.set(0.0);
		}, this).withTimeout(IntakeConstants.kShootDuration);
	}
}
