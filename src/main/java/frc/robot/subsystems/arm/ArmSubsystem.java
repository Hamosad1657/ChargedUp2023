
package frc.robot.subsystems.arm;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.hamosad1657.lib.sensors.HaCANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.arm.ArmConstants.ArmState;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class ArmSubsystem extends SubsystemBase {
	private static ArmSubsystem instance;

	public static ArmSubsystem getInstance() {
		if (instance == null) {
			instance = new ArmSubsystem();
		}
		return instance;
	}

	private final WPI_TalonFX lengthMotor;
	private final CANSparkMax angleMotor;
	private final HaCANCoder angleCANCoder, lengthCANCoder;

	private final ProfiledPIDController anglePIDController;
	private final PIDController lengthPIDController;

	private final DigitalInput extendLimit, retractLimit, bottomAngleLimit, topAngleLimit;

	private boolean angleAtGoal, lengthAtGoal;

	public ArmSubsystem() {
		this.angleMotor = new CANSparkMax(RobotMap.kArmAngleMotorID, MotorType.kBrushless);
		this.angleMotor.setInverted(false);
		this.angleMotor.enableVoltageCompensation(12.0);

		this.angleCANCoder = new HaCANCoder(RobotMap.kArmAngleCANCoderID, ArmConstants.kAngleEncoderOffset);
		this.angleCANCoder.setMeasurmentRange(AbsoluteSensorRange.Unsigned_0_to_360);
		this.angleCANCoder.setPosition(0.0);

		this.lengthCANCoder = new HaCANCoder(RobotMap.kArmLengthCANCoderID, ArmConstants.kLengthEncoderOffset);
		this.lengthCANCoder.setReversed(false);
		this.lengthCANCoder.setMeasurmentRange(AbsoluteSensorRange.Unsigned_0_to_360);
		this.lengthCANCoder.setPosition(0.0);

		this.lengthMotor = new WPI_TalonFX(RobotMap.kArmLengthMotorID);
		this.lengthMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 35.0, 40.0, 0.25));
		this.lengthMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35.0, 40.0, 0.25));
		this.lengthMotor.configNeutralDeadband(0.1);

		this.anglePIDController = ArmConstants.kAnglePIDGains.toProfiledPIDController(ArmConstants.kAnglePIDConstrains);
		this.anglePIDController.setTolerance(ArmConstants.kAngleTolerance);

		this.lengthPIDController = ArmConstants.kArmLengthPIDGains.toPIDController();
		this.lengthPIDController.setTolerance(ArmConstants.kLengthTolerance);

		this.bottomAngleLimit = new DigitalInput(RobotMap.kBottomArmAngleLimitport);
		this.topAngleLimit = new DigitalInput(RobotMap.kTopArmAngleLimitport);
		this.extendLimit = new DigitalInput(RobotMap.kArmExtendLimitPort);
		this.retractLimit = new DigitalInput(RobotMap.kArmRetractLimitPort);

		this.setState(ArmConstants.kAngleMinSetpoint, 0.0);

		if (Robot.showShuffleboardSubsystemInfo) {
			ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

			armTab.add("Arm Length Motor", this.lengthMotor).withPosition(0, 0).withSize(2, 1);
			armTab.add("Arm Angle CANCoder", this.angleCANCoder).withPosition(2, 0).withSize(2, 2);
			armTab.add("Arm Length CANCoder", this.lengthCANCoder).withPosition(4, 0).withSize(2, 2);

			armTab.addBoolean("Extend Limit", () -> !this.extendLimit.get()).withPosition(2, 2).withSize(2, 1);
			armTab.addBoolean("Retract Limit", () -> !this.retractLimit.get()).withPosition(2, 3).withSize(2, 1);
			armTab.addBoolean("Top Angle Limit", () -> !this.topAngleLimit.get()).withPosition(4, 2).withSize(2, 1);
			armTab.addBoolean("Bottom Angle Limit", () -> !this.bottomAngleLimit.get()).withPosition(4, 3).withSize(2,
					1);

			armTab.addDouble("Angle Goal", () -> this.anglePIDController.getGoal().position).withPosition(6, 0)
					.withSize(2, 1);
			armTab.addDouble("Length Setpoint", this.lengthPIDController::getSetpoint).withPosition(6, 1).withSize(2,
					1);
			armTab.addBoolean("Angle At Goal", this.anglePIDController::atGoal).withPosition(6, 2).withSize(2, 1);
			armTab.addBoolean("Length At Setpoint", this.lengthPIDController::atSetpoint).withPosition(6, 3).withSize(2,
					1);
			armTab.add("Subsystem", this).withPosition(0, 1).withSize(2, 3);
			armTab.addDouble("Angle Error", this.anglePIDController::getPositionError);
		}
	}

	public void resetLengthCANCoder() {
		this.lengthCANCoder.setPosition(0.0);
	}

	private double getCurrentLength() {
		return this.lengthCANCoder.getPositionDeg();
	}

	private double getCurrentAngle() {
		return this.angleCANCoder.getAbsAngleDeg();
	}

	/**
	 * @param output - The output of the angle motor in [-1.0, 1.0]. Positive
	 *               output: arm goes up, negative output: arm
	 *               goes down. Doesn't slow near the limits.
	 */
	public void setAngleMotorWithLimits(double output) {
		// The magnetic limit switches are normally true.
		if ((output > 0.0 && !this.topAngleLimit.get()) || (output < 0.0 && !this.bottomAngleLimit.get())) {
			this.angleMotor.set(0.0);
		} else {
			this.angleMotor.set(output);
		}
	}

	/**
	 * @param output - The output of the length motor in [-1.0, 1.0]. Positive
	 *               output extends, negative output retracts.
	 *               Doesn't slow near the limits.
	 */
	public void setLengthMotorWithLimits(double output) {
		// The magnetic limit switches are normally true.
		if ((output < 0.0 && !this.extendLimit.get()) || (output > 0.0 && !this.retractLimit.get())) {
			this.lengthMotor.set(0.0);
		} else {
			this.lengthMotor.set(output);
		}
	}

	/**
	 * @return The output for the angle motor calculated by the PID in [-1.0, 1.0].
	 */
	public double calculateAngleMotorOutput() {
		double output = this.anglePIDController.calculate(this.getCurrentAngle());
		output = MathUtil.clamp(output, -ArmConstants.kAngleMotorMaxPIDOutput, ArmConstants.kAngleMotorMaxPIDOutput);

		if (output < 0.0) {
			output *= ArmConstants.kAngleDownOutputRatio;
		}

		return output;
	}

	/**
	 * @return The output for the length motor calculated by the PID in [-1.0, 1.0].
	 */
	public double calculateLengthMotorOutput() {
		double output = -this.lengthPIDController.calculate(this.getCurrentLength());
		return MathUtil.clamp(output, -ArmConstants.kLengthMotorMaxOutput, ArmConstants.kLengthMotorMaxOutput);
	}

	/**
	 * Sets the setpoint of the arm and length motors to the new arm state.
	 * 
	 * @param angleDeg  - The new state's angle.
	 * @param lengthDeg - The new state's length.
	 */
	public void setState(double angleDeg, double lengthDeg) {
		this.anglePIDController.setGoal(angleDeg);
		this.lengthPIDController.setSetpoint(lengthDeg);
	}

	/**
	 * Sets the setpoint of the arm and length motors to the new arm state.
	 * 
	 * @param newState - The new arm state.
	 */
	public void setState(ArmState newState) {
		this.setState(newState.angleDeg, newState.lengthDeg);
	}

	public Command setStateCommand(ArmState newState) {
		return new InstantCommand(() -> this.setState(newState));
	}

	public Command getToStateCommand(ArmState newState, boolean endAtSetpoint) {
		return new FunctionalCommand(() -> {
			this.angleAtGoal = false;
			this.setState(newState);
		}, () -> {
			this.setAngleMotorWithLimits(this.calculateAngleMotorOutput());
			if (this.anglePIDController.atGoal()) {
				this.angleAtGoal = true;
			}

			if (this.angleAtGoal) {
				this.setLengthMotorWithLimits(this.calculateLengthMotorOutput());
			}
		}, (interrupted) -> {
			this.setLengthMotorWithLimits(0.0);
		}, () -> (endAtSetpoint ? (this.angleAtGoal && this.lengthPIDController.atSetpoint()) : this.joysticksMoved()),
				this);
	}

	public Command getToStateCommand(ArmState newState) {
		return this.getToStateCommand(newState, false);
	}

	public Command getToStateLengthFirstCommand(ArmState newState, boolean endAtSetpoint) {
		return new FunctionalCommand(() -> {
			this.lengthAtGoal = false;
			this.setState(newState);
		}, () -> {
			if (this.lengthPIDController.atSetpoint()) {
				this.lengthAtGoal = true;
			}

			if (this.lengthAtGoal) {
				this.setLengthMotorWithLimits(ArmConstants.kHomingLengthKeepRetractedOutput / 2.0);
				this.setAngleMotorWithLimits(this.calculateAngleMotorOutput());
			} else {
				this.setLengthMotorWithLimits(this.calculateLengthMotorOutput());
			}
		}, (interrupted) ->

		{
			this.setLengthMotorWithLimits(0.0);
		}, () -> (endAtSetpoint ? (this.lengthAtGoal && this.anglePIDController.atGoal()) : this.joysticksMoved()),
				this);
	}

	public Command getToStateLengthFirstCommand(ArmState newState) {
		return this.getToStateCommand(newState, false);
	}

	/**
	 * Returns a RunCommand to manually control the arm's length and angle using 3
	 * output suppliers.
	 * 
	 * @param angleOutputSupplier     - The output supplier for the angle motor.
	 * @param forwardsOutputSupplier  - The output supplier for the arm's forwards
	 *                                extension.
	 * @param backwardsOutputSupplier - The output supplier for the arm's backwards
	 *                                retraction.
	 */
	public Command openLoopTeleopCommand(DoubleSupplier angleOutputSupplier, DoubleSupplier forwardsOutputSupplier,
			DoubleSupplier backwardsOutputSupplier) {
		return new RunCommand(() -> {
			// Set angle motor
			this.setAngleMotorWithLimits(angleOutputSupplier.getAsDouble() * -ArmConstants.kAngleMotorMaxOutput);

			// Set length motors
			double lengthSupplierValue = backwardsOutputSupplier.getAsDouble() - forwardsOutputSupplier.getAsDouble();
			this.setLengthMotorWithLimits(lengthSupplierValue * ArmConstants.kLengthMotorMaxOutput);
		}, this);
	}

	/**
	 * Returns a RunCommand to manually change the arm's state using 3 output
	 * suppliers.
	 * 
	 * @param angleOutputSupplier     - The output supplier for the angle motor.
	 * @param forwardsOutputSupplier  - The output supplier for the arm's forwards
	 *                                extension.
	 * @param backwardsOutputSupplier - The output supplier for the arm's backwards
	 *                                retraction.
	 */
	public Command closedLoopTeleopCommand(DoubleSupplier angleOutputSupplier, DoubleSupplier forwardsOutputSupplier,
			DoubleSupplier backwardsOutputSupplier) {
		return new RunCommand(() -> {
			// Set angle motor
			double newAngleSetpoint = this.anglePIDController.getGoal().position
					+ angleOutputSupplier.getAsDouble() * -ArmConstants.kAngleTeleopSetpointMultiplier;
			newAngleSetpoint = MathUtil.clamp(newAngleSetpoint, ArmConstants.kAngleMinSetpoint,
					ArmConstants.kAngleMaxSetpoint);
			this.anglePIDController.setGoal(newAngleSetpoint);
			this.setAngleMotorWithLimits(this.calculateAngleMotorOutput());

			// Set length motors
			double lengthSupplierValue = forwardsOutputSupplier.getAsDouble() - backwardsOutputSupplier.getAsDouble();
			double newLengthSetpoint = this.lengthPIDController.getSetpoint()
					+ lengthSupplierValue * ArmConstants.kLengthTeleopSetpointMultiplier;
			newLengthSetpoint = MathUtil.clamp(newLengthSetpoint, ArmConstants.kLengthMinSetpoint,
					ArmConstants.kLengthMaxSetpoint);
			this.lengthPIDController.setSetpoint(newLengthSetpoint);
			this.setLengthMotorWithLimits(this.calculateLengthMotorOutput());
		}, this);
	}

	public Command autoHomeCommand() {
		return new RunCommand(() -> {
			if (this.getCurrentAngle() < ArmConstants.kLengthRetractMinAngle) {
				this.setAngleMotorWithLimits(-ArmConstants.kHomingAngleOutput);
			}
		}, this).until(() -> this.getCurrentAngle() > ArmConstants.kLengthRetractMinAngle)
				.andThen(new RunCommand(() -> {
					// The limits are normally true
					if (this.retractLimit.get()) {
						this.setLengthMotorWithLimits(ArmConstants.kHomingLengthOutput);
						this.setAngleMotorWithLimits(0.0);
						return;
					}

					this.setLengthMotorWithLimits(0.0);
					if (this.bottomAngleLimit.get()) {
						this.setState(ArmConstants.kAngleMinSetpoint, 0.0);
						this.setAngleMotorWithLimits(
								this.calculateAngleMotorOutput() * ArmConstants.kHomingAnglePIDRatio);
					} else {
						this.setAngleMotorWithLimits(0.0);
					}
				}, this).until(
						() -> (!this.retractLimit.get() && !this.bottomAngleLimit.get()) || this.joysticksMoved())
						.finallyDo((interrupted) -> {
							if (!(interrupted || this.joysticksMoved())) {
								this.resetLengthCANCoder();
								this.anglePIDController.reset(this.getCurrentAngle());
								this.lengthPIDController.reset();
							}
							this.setState(this.getCurrentAngle(), this.getCurrentLength());
						}));
	}

	public Command homeCommand() {
		return this.autoHomeCommand().andThen(new RunCommand(() -> {
			this.setAngleMotorWithLimits(ArmConstants.kHomingAngleOutput);
			this.setLengthMotorWithLimits(ArmConstants.kHomingLengthKeepRetractedOutput);
		}, this).until(this::joysticksMoved));
	}

	public Command pickupConeCommand() {
		return new SequentialCommandGroup(this.getToStateCommand(ArmState.kLowCone, true),
				GrabberSubsystem.getInstance().collectCommand(), this.getToStateCommand(ArmState.kLowConePickup));
	}

	public Command retractCommand() {
		return new StartEndCommand(() -> this.setLengthMotorWithLimits(ArmConstants.kHomingLengthOutput), () -> {
			this.setLengthMotorWithLimits(0.0);
			this.resetLengthCANCoder();
		}, this).until(() -> !this.retractLimit.get());
	}

	public boolean joysticksMoved() {
		double lengthValue = (RobotContainer.driverB_Controller.getL2Axis() + 1.0)
				- (RobotContainer.driverB_Controller.getR2Axis() + 1.0);
		double angleValue = RobotContainer.driverB_Controller.getLeftY();

		return (lengthValue > RobotContainer.kJoystickDeadband || lengthValue < -RobotContainer.kJoystickDeadband
				|| angleValue > RobotContainer.kJoystickDeadband || angleValue < -RobotContainer.kJoystickDeadband);
	}

	public void setAngleIdleMode(IdleMode idleMode) {
		this.angleMotor.setIdleMode(idleMode);
	}

	public void setLengthIdleMode(IdleMode idleMode) {
		if (idleMode == IdleMode.kBrake) {
			this.lengthMotor.setNeutralMode(NeutralMode.Brake);
		} else {
			this.lengthMotor.setNeutralMode(NeutralMode.Coast);
		}
	}
}
