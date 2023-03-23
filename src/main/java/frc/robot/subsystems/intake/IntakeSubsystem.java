
package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.hamosad1657.lib.sensors.HaCANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
	private final TalonFXSensorCollection intakeSensors;
	private final DigitalInput raiseLimit, lowerLimit;

	private boolean isIntakeLowered;
	private ShootHeight currentShootHeight;

	private IntakeSubsystem() {
		this.angleMotor = new CANSparkMax(RobotMap.kIntakeAngleMotorID, MotorType.kBrushless);
		this.angleMotor.setIdleMode(IdleMode.kBrake);

		this.angleController = IntakeConstants.kAngleMotorGains.toPIDController();
		this.angleController.setTolerance(IntakeConstants.kAngleTolerance);

		this.angleCANCoder = new HaCANCoder(RobotMap.kIntakeAngleCANCoderID, IntakeConstants.kAngleCANCoderOffset);
		this.angleCANCoder.setMeasurmentRange(AbsoluteSensorRange.Unsigned_0_to_360);
		this.angleCANCoder.setReversed(true);

		this.intakeMotor = new WPI_TalonFX(RobotMap.kIntakeMotorID);
		this.intakeMotor.setNeutralMode(NeutralMode.Coast);
		this.intakeMotor.setInverted(true);
		this.intakeMotor.configVoltageCompSaturation(12.0);
		this.intakeMotor.enableVoltageCompensation(true);
		this.intakeSensors = this.intakeMotor.getSensorCollection();
		/** Wired normally true, false when pressed. */
		this.raiseLimit = new DigitalInput(RobotMap.kIntakeRaiseLimitPort);
		/** Wired normally true, false when pressed. */
		this.lowerLimit = new DigitalInput(RobotMap.kIntakeLowerLimitPort);

		this.isIntakeLowered = false;
		this.currentShootHeight = ShootHeight.kFar;

		if (Robot.showShuffleboardSubsystemInfo) {
			ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");

			intakeTab.addBoolean("Raise Limit", () -> !this.raiseLimit.get()).withPosition(0, 0).withSize(1, 1);
			intakeTab.addBoolean("Lower Limit", () -> !this.lowerLimit.get()).withPosition(1, 0).withSize(1, 1);

			intakeTab.addDouble("Intake Angle", this.angleCANCoder::getAbsAngleDeg).withPosition(2, 0).withSize(1, 1);
			intakeTab.addDouble("Intake Setpoint", this.angleController::getSetpoint).withPosition(3, 0).withSize(1, 1);
			intakeTab.addString("Shoot Height", () -> this.currentShootHeight.name()).withPosition(4, 0).withSize(1, 1);
			intakeTab.addDouble("Intake Angle Error", this.angleController::getPositionError).withPosition(0, 1)
					.withSize(3, 3).withWidget(BuiltInWidgets.kGraph);
			intakeTab.addBoolean("Intake Angle At Setpoint", this.angleController::atSetpoint).withPosition(3, 1)
					.withSize(2, 1);

			intakeTab.addDouble("Intake Motor Output", this.intakeMotor::get).withPosition(3, 2).withSize(2, 1);
			intakeTab.addDouble("Intake Motor Velocity", this::getIntakeVelocity)
					.withWidget(BuiltInWidgets.kGraph)
					.withPosition(5, 0).withSize(4, 4);

		}
	}

	/**
	 * @return The speed of the intake motor shaft, in encoder ticks (2048) per
	 *         second.
	 */
	public double getIntakeVelocity() {
		return this.intakeSensors.getIntegratedSensorVelocity() / 10.0;
	}

	public void setAngleIdleMode(IdleMode idleMode) {
		this.angleMotor.setIdleMode(idleMode);
	}

	/**
	 * Sets the output for the angle motor. If a limit is pressed and the output is
	 * for the direction of the limit, set 0.
	 * 
	 * @param output - Percent output 1 to -1.
	 */
	public void setAngleMotorWithLimits(double output) {
		if ((output > 0.0 && !this.raiseLimit.get()) || (output < 0.0 && !this.lowerLimit.get())) {
			this.angleMotor.set(0.0);
		} else {
			this.angleMotor.set(output);
		}
	}

	/**
	 * @return The PID output (according to the angle setpoint) in percent output 1
	 *         to -1.
	 */
	public double calculateAngleMotorOutput() {
		double output = this.angleController.calculate(this.angleCANCoder.getAbsAngleDeg());

		if (this.angleController.atSetpoint()) {
			return 0.0;
		}

		output = MathUtil.clamp(output,
				-IntakeConstants.kAngleMotorMaxPIDOutput,
				IntakeConstants.kAngleMotorMaxPIDOutput);
		return output;
	}

	/** Lowers the intake untill the limit switch is pressed. */
	public Command lowerCommand() {
		return new StartEndCommand(() -> {
			this.setAngleMotorWithLimits(-IntakeConstants.kAngleMotorDefaultOutput);
			this.intakeMotor.set(IntakeConstants.kIntakeMotorCollectOutput);
		}, () -> {
			this.intakeMotor.set(0.0);
			this.setAngleMotorWithLimits(0.0);
			this.isIntakeLowered = true;
		}, this).until(
				() -> !this.lowerLimit.get() && this.getIntakeVelocity() < IntakeConstants.kCollectWithCubeMaxVelocity);
	}

	/** Raises the intake untill the limit switch is pressed. */
	public Command raiseCommand() {
		return new StartEndCommand(() -> {
			this.setAngleMotorWithLimits(IntakeConstants.kAngleMotorDefaultOutput);
		}, () -> {
			this.setAngleMotorWithLimits(0.0);
			this.intakeMotor.set(0.0);
			this.isIntakeLowered = false;
		}, this).until(() -> !this.raiseLimit.get());
	}

	/** If the intake is supposed to be raised, apply power to keep it that way. */
	public Command keepInPlaceCommand() {
		return new RunCommand(
				() -> {
					if (this.isIntakeLowered) {
						this.angleMotor.set(IntakeConstants.kAngleMotorKeepInPlaceOutput * -3.0);
					} else {
						this.angleMotor.set(IntakeConstants.kAngleMotorKeepInPlaceOutput);
					}
				},
				this);
	}

	public Command getToShootHeightCommand(ShootHeight shootHeight) {
		return new FunctionalCommand(() -> {
			this.currentShootHeight = shootHeight;
			this.intakeMotor.set(0.0);

			this.angleController.reset();
			this.angleController.setSetpoint(shootHeight.angle);
		}, () -> {
			this.setAngleMotorWithLimits(this.calculateAngleMotorOutput());
		}, (interrupted) -> {
			this.setAngleMotorWithLimits(0.0);
		}, () -> {
			return false;
		}, this);
	}

	/**
	 * Shoots a cube and then raises the intake. This assumes that the angle of the
	 * intake is already correct.
	 * 
	 * @param isAuto - Should the command use auto raise command or teleop raise
	 *               command.
	 */
	public Command shootCommand(boolean isAuto) {
		return new RunCommand(() -> {
			this.intakeMotor.set(IntakeConstants.kIntakeMotorCollectOutput);
		}, this)
				.withTimeout(IntakeConstants.kShootCollectDuration)
				.andThen(
						new StartEndCommand(() -> {
							this.intakeMotor.set(this.currentShootHeight.motorOutput);
						}, () -> {
							this.intakeMotor.set(0.0);
						}, this)
								.withTimeout(IntakeConstants.kShootDuration))
				.andThen((isAuto ? this.autoRaiseCommand() : this.raiseCommand()).withTimeout(1.0));
	}

	/**
	 * Shoots a cube and then raises the intake. This assumes that the angle of the
	 * intake is already correct.
	 * 
	 * Defaults to teleop raise after the shooting.
	 */
	public Command shootCommand() {
		return this.shootCommand(false);
	}

	public Command autoCollectPieceCommand() {
		return new StartEndCommand(() -> {
			this.setAngleMotorWithLimits(-IntakeConstants.kAngleMotorDefaultOutput);
			this.intakeMotor.set(IntakeConstants.kIntakeMotorCollectOutput);
		}, () -> {
			this.setAngleMotorWithLimits(IntakeConstants.kAngleMotorKeepInPlaceOutput * -3.0);
		}, this).until(() -> !this.lowerLimit.get()).andThen(
				new StartEndCommand(() -> {
				}, () -> {
					this.intakeMotor.set(0.0);
				}, this).withTimeout(10.0));
	}

	public Command autoRaiseCommand() {
		return new StartEndCommand(() -> {
			this.setAngleMotorWithLimits(IntakeConstants.kAngleMotorDefaultOutput * 1.35);
		}, () -> {
			this.setAngleMotorWithLimits(0.0);
			this.isIntakeLowered = false;
		}, this).until(() -> !this.raiseLimit.get());
	}
}
