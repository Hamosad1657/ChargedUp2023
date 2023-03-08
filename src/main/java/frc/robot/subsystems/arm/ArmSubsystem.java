
package frc.robot.subsystems.arm;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.hamosad1657.lib.motors.HaTalonSRX;
import com.hamosad1657.lib.sensors.HaCANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.subsystems.arm.ArmConstants.ArmState;

public class ArmSubsystem extends SubsystemBase {
	private static ArmSubsystem instance;

	public static ArmSubsystem getInstance() {
		if (instance == null) {
			instance = new ArmSubsystem();
		}
		return instance;
	}

	// Only used to keep the motors' objects - Use [armLengthMotor] for control.
	private final WPI_TalonSRX lengthMotorA, lengthMotorB;

	private final CANSparkMax angleMotor;
	private final HaTalonSRX lengthMotor;
	private final HaCANCoder angleCANCoder, lengthCANCoder;

	private final ProfiledPIDController anglePIDController;
	private final PIDController lengthPIDController;

	private final DigitalInput extendLimit, retractLimit, bottomAngleLimit, topAngleLimit;

	public ArmSubsystem() {
		this.angleMotor = new CANSparkMax(RobotMap.kArmAngleMotorID, MotorType.kBrushless);
		this.angleMotor.setIdleMode(IdleMode.kBrake);
		this.angleMotor.setInverted(false);
		this.angleMotor.enableVoltageCompensation(12.0);

		this.angleCANCoder = new HaCANCoder(RobotMap.kArmAngleCANCoderID, ArmConstants.kAngleEncoderOffset);
		this.angleCANCoder.setMeasurmentRange(AbsoluteSensorRange.Unsigned_0_to_360);
		this.angleCANCoder.setPosition(0.0);

		this.lengthMotorA = new WPI_TalonSRX(RobotMap.kArmLengthMotorAID);
		this.lengthMotorA.setNeutralMode(NeutralMode.Brake);
		this.lengthMotorB = new WPI_TalonSRX(RobotMap.kArmLengthMotorBID);
		this.lengthMotorB.setNeutralMode(NeutralMode.Brake);
		this.lengthMotorB.follow(this.lengthMotorA);
		this.lengthMotor = new HaTalonSRX(this.lengthMotorA);

		this.lengthCANCoder = new HaCANCoder(RobotMap.kArmLengthCANCoderID, ArmConstants.kLengthEncoderOffset);
		this.lengthCANCoder.setReversed(true);
		this.lengthCANCoder.setMeasurmentRange(AbsoluteSensorRange.Unsigned_0_to_360);
		this.lengthCANCoder.setPosition(0.0);

		this.anglePIDController = ArmConstants.kAnglePIDGains.toProfiledPIDController(ArmConstants.kAnglePIDConstrains);
		this.anglePIDController.setTolerance(ArmConstants.kAngleTolerance);

		this.lengthPIDController = ArmConstants.kArmLengthPIDGains.toPIDController();
		this.lengthPIDController.setTolerance(ArmConstants.kLengthTolerance);

		this.bottomAngleLimit = new DigitalInput(RobotMap.kBottomArmAngleLimitport);
		this.topAngleLimit = new DigitalInput(RobotMap.kTopArmAngleLimitport);
		this.extendLimit = new DigitalInput(RobotMap.kArmExtendLimitPort);
		this.retractLimit = new DigitalInput(RobotMap.kArmRetractLimitPort);

		ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

		armTab.add("Arm Length Motor", this.lengthMotor).withPosition(0, 0).withSize(2, 5);
		armTab.add("Arm Angle CANCoder", this.angleCANCoder).withPosition(2, 0).withSize(2, 2);
		armTab.add("Arm Length CANCoder", this.lengthCANCoder).withPosition(4, 0).withSize(2, 2);

		armTab.addBoolean("Extend Limit", () -> !this.extendLimit.get()).withPosition(2, 2).withSize(2, 1);
		armTab.addBoolean("Retract Limit", () -> !this.retractLimit.get()).withPosition(2, 3).withSize(2, 1);
		armTab.addBoolean("Top Angle Limit", () -> !this.topAngleLimit.get()).withPosition(4, 2).withSize(2, 1);
		armTab.addBoolean("Bottom Angle Limit", () -> !this.bottomAngleLimit.get()).withPosition(4, 3).withSize(2, 1);

		armTab.addDouble("Length Error", this.lengthPIDController::getPositionError);
		armTab.addDouble("Current Length", this::getCurrentLength);
		armTab.addDouble("Length Motor Output", this.lengthMotor::get);
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
	 * @param output - The output of the angle motor in [-1.0, 1.0]. Positive output: arm goes up, negative output: arm
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
	 * @param output - The output of the length motor in [-1.0, 1.0]. Positive output extends, negative output retracts.
	 *               Doesn't slow near the limits.
	 */
	public void setLengthMotorWithLimits(double output) {
		// The magnetic limit switches are normally true.
		if (output < 0.0 && !this.extendLimit.get()) {
			this.lengthMotor.set(0.0);
		} else if (output > 0.0 && !this.retractLimit.get()) {
			this.lengthMotor.set(0.0);
		} else {
			this.lengthMotor.set(output);
		}
	}

	/**
	 * @return The output for the angle motor calculated by the PID in [-1.0, 1.0].
	 */
	public double calculateAngleMotorOutput() {
		double output = MathUtil.clamp(this.anglePIDController.calculate(this.getCurrentAngle()),
				-ArmConstants.kAngleMotorMaxPIDOutput, ArmConstants.kAngleMotorMaxPIDOutput);

		if (output < 0.0) {
			output *= ArmConstants.kAngleDownOutputRatio;
		}

		return output;
	}

	/**
	 * @return The output for the length motor calculated by the PID in [-1.0, 1.0].
	 */
	public double calculateLengthMotorOutput() {
		double output = -MathUtil.clamp(this.lengthPIDController.calculate(this.getCurrentLength()),
				-ArmConstants.kLengthMotorMaxOutput, ArmConstants.kLengthMotorMaxOutput);
		return output;
	}

	/**
	 * @return If both the angle and length motors are at their setpoint.
	 */
	public boolean isAtSetpoint() {
		return this.anglePIDController.atSetpoint() && this.lengthPIDController.atSetpoint();
	}

	/**
	 * Sets the setpoint of the arm and length motors to the new arm state.
	 * 
	 * @param newState - The new arm state.
	 */
	public void setState(ArmState newState) {
		this.anglePIDController.setGoal(newState.angleDeg);
		this.lengthPIDController.setSetpoint(newState.lengthDeg);
	}

	public Command getToStateCommand(ArmState newState) {
		return new FunctionalCommand(() -> this.setState(newState), () -> {
			this.setAngleMotorWithLimits(this.calculateAngleMotorOutput());
			if (this.getCurrentAngle() > ArmConstants.kLengthExtendMinAngle) {
				this.setLengthMotorWithLimits(this.calculateLengthMotorOutput());
			}
		}, (interrupted) -> {
		}, this::shoudlArmMove, this);
	}

	/**
	 * Returns a RunCommand to manually control the arm's length and angle using 3 output suppliers.
	 * 
	 * @param angleOutputSupplier     - The output supplier for the angle motor.
	 * @param forwardsOutputSupplier  - The output supplier for the arm's forwards extension.
	 * @param backwardsOutputSupplier - The output supplier for the arm's backwards retraction.
	 */
	public Command openLoopTeleopCommand(DoubleSupplier angleOutputSupplier, DoubleSupplier forwardsOutputSupplier,
			DoubleSupplier backwardsOutputSupplier, PS4Controller controller) {
		return new RunCommand(() -> {
			// Set angle motor
			this.setAngleMotorWithLimits(angleOutputSupplier.getAsDouble() * -ArmConstants.kAngleMotorMaxOutput);

			// Set length motors
			double lengthSupplierValue = backwardsOutputSupplier.getAsDouble() - forwardsOutputSupplier.getAsDouble();
			this.setLengthMotorWithLimits(lengthSupplierValue * ArmConstants.kLengthMotorMaxOutput);
		}, this);
	}

	public Command homeCommand() {
		return new FunctionalCommand(() -> {
		}, () -> {
			// The limits are normally true
			if (this.retractLimit.get()) {
				this.setLengthMotorWithLimits(ArmConstants.kHomingLengthOutput);
				this.setAngleMotorWithLimits(0.0);
			} else {
				this.setLengthMotorWithLimits(0.0);
				if (this.bottomAngleLimit.get()) {
					this.setAngleMotorWithLimits(ArmConstants.kHomingAngleOutput);
				} else {
					this.angleMotor.set(0.0);
				}
			}

		}, (interrupted) -> {
			Robot.print("Finished homing arm");
			if (!interrupted && !this.shoudlArmMove()) {
				this.lengthCANCoder.setPosition(0.0);
				this.angleCANCoder.setPosition(0.0);
			}
		}, () -> (!this.retractLimit.get() && !this.bottomAngleLimit.get()) || this.shoudlArmMove(), this);
	}

	public boolean shoudlArmMove() {
		double lengthValue = (RobotContainer.driverB_Controller.getL2Axis() + 1.0)
				- (RobotContainer.driverB_Controller.getR2Axis() + 1.0);
		double angleValue = RobotContainer.driverB_Controller.getLeftY();

		return (lengthValue > RobotContainer.kJoystickDeadband || lengthValue < -RobotContainer.kJoystickDeadband
				|| angleValue > RobotContainer.kJoystickDeadband || angleValue < -RobotContainer.kJoystickDeadband);
	}
}
