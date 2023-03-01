
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
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmConstants.ArmState;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

public class ArmSubsystem extends SubsystemBase {
	private static ArmSubsystem instance;

	public static ArmSubsystem getInstance() {
		if (instance == null) {
			instance = new ArmSubsystem();
		}
		return instance;
	}

	private final CANSparkMax armAngleMotor;
	private final WPI_TalonSRX armLengthMotorA, armLengthMotorB; // Only used to keep the motors' objects - Use
																	// [armLengthMotor] for control.

	/** The object to use for controlling both length motors. */
	private final HaTalonSRX armLengthMotor;
	private final HaCANCoder armAngleCANCoder, armLengthCANCoder;
	private final PIDController armLengthPIDController, anglePIDController;
	private final ArmFeedforward armAngleFFController;
	private final DigitalInput extendLimit, retractLimit, bottomAngleLimit, topAngleLimit;
	private final GenericEntry extendLimitEntry, retractLimitEntry, topAngleLimitEntry, bottomAngleLimitEntry,
			angleMotorOutputEntry;
	private ArmState currentState;
	private boolean shouldOverrideLimits = false;

	// Angle lowers when you go up
	public ArmSubsystem() {
		this.armAngleMotor = new CANSparkMax(RobotMap.kArmAngleMotorID, MotorType.kBrushless);
		this.armAngleMotor.setIdleMode(IdleMode.kBrake);
		this.armAngleMotor.setInverted(true);
		this.armAngleMotor.enableVoltageCompensation(12.0);
		this.armAngleCANCoder = new HaCANCoder(RobotMap.kArmAngleCANCoderID, ArmConstants.kAngleCANCoderOffsetDeg);
		this.armAngleCANCoder.setMeasurmentRange(AbsoluteSensorRange.Unsigned_0_to_360);
		this.armAngleCANCoder.setPosition(0);

		this.armLengthMotorA = new WPI_TalonSRX(RobotMap.kArmLengthMotorAID);
		this.armLengthMotorA.setNeutralMode(NeutralMode.Brake);
		this.armLengthMotorB = new WPI_TalonSRX(RobotMap.kArmLengthMotorBID);
		this.armLengthMotorB.setNeutralMode(NeutralMode.Brake);
		this.armLengthMotorB.follow(this.armLengthMotorA);

		this.armLengthMotor = new HaTalonSRX(armLengthMotorA);
		this.armLengthCANCoder = new HaCANCoder(RobotMap.kArmLengthCANCoderID, ArmConstants.kArmLengthEncoderOffset);
		this.armLengthCANCoder.setMeasurmentRange(AbsoluteSensorRange.Unsigned_0_to_360);
		this.armLengthCANCoder.setPosition(0);

		this.anglePIDController = ArmConstants.kArmAnglePIDGains.toPIDController();
		this.anglePIDController.setTolerance(ArmConstants.kArmAngleTolerance);
		// TODO: Implement arm angle FF control
		this.armAngleFFController = new ArmFeedforward(ArmConstants.kArmAngleFFkS, ArmConstants.kArmAngleFFkG,
				ArmConstants.kArmAngleFFkV);

		this.armLengthPIDController = ArmConstants.kArmLengthPIDGains.toPIDController();
		this.armLengthPIDController.setTolerance(ArmConstants.kArmLengthTolerance);

		this.extendLimit = new DigitalInput(RobotMap.kArmExtendLimitPort);
		this.retractLimit = new DigitalInput(RobotMap.kArmRetractLimitPort);
		this.bottomAngleLimit = new DigitalInput(RobotMap.kBottomArmAngleLimitport);
		this.topAngleLimit = new DigitalInput(RobotMap.kTopArmAngleLimitport);

		ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
		// armTab.add("Arm Angle Motor", this.armAngleMotor);
		armTab.add("Arm Length Motor", this.armLengthMotor);

		armTab.add("Arm Length CANCoder", this.armLengthCANCoder).withPosition(4, 0).withSize(2, 2);
		this.extendLimitEntry = armTab.add("Extend Limit", false).withPosition(4, 2).withSize(2, 1).getEntry();
		this.retractLimitEntry = armTab.add("Retract Limit", false).withPosition(4, 3).withSize(2, 1).getEntry();

		armTab.add("Arm Angle CANCoder", this.armAngleCANCoder).withPosition(7, 0).withSize(2, 2);
		this.topAngleLimitEntry = armTab.add("Top Angle Limit", false).withPosition(7, 2).withSize(2, 1).getEntry();
		this.bottomAngleLimitEntry = armTab.add("Bottom Angle Limit", false).withPosition(7, 3).withSize(2, 1)
				.getEntry();

		this.angleMotorOutputEntry = armTab.add("Angle Motor Output", 0.0).withPosition(10, 0).withSize(2, 1)
				.getEntry();
	}

	private double getCurrentLength() {
		return this.armLengthCANCoder.getPositionDeg();
	}

	private double getCurrentAngle() {
		return this.armAngleCANCoder.getAbsAngleDeg();
	}

	/**
	 * Sets the setpoint of the arm and length motors to the new arm state.
	 * 
	 * @param newState - The new arm state.
	 */
	public void setState(ArmState newState) {
		this.anglePIDController.setSetpoint(newState.angleDeg);
		this.armLengthPIDController.setSetpoint(newState.lengthDeg);
	}

	/**
	 * @return The output for the angle motor calculated by the PID in [-1.0, 1.0].
	 */
	public double calculateAngleMotorOutput() {
		double output = -MathUtil.clamp(this.anglePIDController.calculate(this.getCurrentAngle()),
				-ArmConstants.kArmAngleSpeedRatio, ArmConstants.kArmAngleSpeedRatio);
		return output;
	}

	/**
	 * @return The output for the length motor calculated by the PID in [-1.0, 1.0].
	 */
	public double calculateLengthMotorOutput() {
		double output = -MathUtil.clamp(this.armLengthPIDController.calculate(this.getCurrentLength()),
				-ArmConstants.kMaxMotorOutput, ArmConstants.kMaxMotorOutput);
		return output;
	}

	/**
	 * @param output - The output of the angle motor in [-1.0, 1.0]. Positive output: arm goes down, negative output:
	 *               arm goes up. Slows near the limits.
	 */
	public void setAngleMotorWithThresholdsAndLimits(double output) {
		// The magnetic limit switches are normally true.
		if ((output < 0.0 && !this.topAngleLimit.get()) || (output > 0.0 && !this.bottomAngleLimit.get())) {
			this.angleMotorOutputEntry.setDouble(0.0);
			this.armAngleMotor.set(0.0);
		} else {
			double armAngle = this.getCurrentAngle();
			if ((output > -ArmConstants.kArmAngleBalanceMiddleMotorOutput
					&& output < ArmConstants.kArmAngleBalanceMiddleMotorOutput)) {
				if (armAngle > ArmConstants.kArmAngleBalanceMiddleDeg) {
					output = -ArmConstants.kArmAngleBalanceMiddleMotorOutput;
				} else if (armAngle > ArmConstants.kArmAngleBalanceMinDeg) {
					output = -ArmConstants.kArmAngleBalanceMinMotorOutput;
				} else {
					output = ArmConstants.kArmAngleBalanceMinMotorOutput;
				}
			} else {
				// Make the angle motor slower at the top and bottom
				if (armAngle < ArmConstants.kArmAngleBottomThreshold || armAngle > ArmConstants.kArmAngleTopThreshold) {
					output *= ArmConstants.kArmAngleThresholdSpeedRatio;
				}
			}

			this.angleMotorOutputEntry.setDouble(output);
			this.armAngleMotor.set(output);
		}
	}

	/**
	 * @param output - The output of the angle motor in [-1.0, 1.0]. Positive output: arm goes down, negative output:
	 *               arm goes up. Doesn't slow near the limits.
	 */
	public void setAngleMotorWithLimits(double output) {
		// The magnetic limit switches are normally true.
		if ((output < 0.0 && !this.topAngleLimit.get()) || (output > 0.0 && !this.bottomAngleLimit.get())) {
			this.angleMotorOutputEntry.setDouble(0.0);
			this.armAngleMotor.set(0.0);
		} else {
			double armAngle = this.getCurrentAngle();
			if ((output > -ArmConstants.kArmAngleBalanceMiddleMotorOutput
					&& output < ArmConstants.kArmAngleBalanceMiddleMotorOutput)) {
				if (armAngle > ArmConstants.kArmAngleBalanceMiddleDeg) {
					output = -ArmConstants.kArmAngleBalanceMiddleMotorOutput;
				} else if (armAngle > ArmConstants.kArmAngleBalanceMinDeg) {
					output = -ArmConstants.kArmAngleBalanceMinMotorOutput;
				} else {
					output = ArmConstants.kArmAngleBalanceMinMotorOutput;
				}
			}
			this.angleMotorOutputEntry.setDouble(output);
			this.armAngleMotor.set(output);
		}
	}

	/**
	 * @param output - The output of the length motor in [-1.0, 1.0]. Positive output extends, negative output retracts.
	 *               Slows near the limits.
	 */
	public void setLengthMotorWithThresholdsAndLimits(double output) {
		// The magnetic limit switches are normally true.
		if (!this.shouldOverrideLimits && output < 0 && !this.extendLimit.get()) {
			this.armLengthMotor.set(0.0);
		} else if (!this.shouldOverrideLimits && output > 0 && !this.retractLimit.get()) {
			this.armLengthMotor.set(0.0);
		} else {
			// Make the length motor slower at the ends of the telescopic
			double armLengthPosition = this.getCurrentLength();
			if (armLengthPosition > ArmConstants.kArmLengthExtendThreshold
					|| armLengthPosition < ArmConstants.kArmLengthRetractThreshold) {
				output *= ArmConstants.kArmLengthThresholdSpeedRatio;
			}
			this.armLengthMotor.set(output);
		}
	}

	/**
	 * @param output - The output of the length motor in [-1.0, 1.0]. Positive output extends, negative output retracts.
	 *               Doesn't slow near the limits.
	 */
	public void setLengthMotorWithLimits(double output) {
		if (output < 0 && !this.extendLimit.get()) { // The magnetic limit switches are normally true.
			this.armLengthMotor.set(0.0);
		} else if (output > 0 && !this.retractLimit.get()) {
			this.armLengthMotor.set(0.0);
		} else {
			this.armLengthMotor.set(output);
		}
	}

	public void setAngleMotorWithLengthLimits(double output) {
		// Only go down if the arm is retracted enough to allow it
		if (output > 0.0) {
			if (this.getCurrentLength() < ArmConstants.maxLengthForAngle(this.getCurrentAngle())) {
				this.setAngleMotorWithLimits(output);
			} else {
				this.setAngleMotorWithLimits(0.0);
			}
		} else {
			this.setAngleMotorWithLimits(output);
		}
	}

	public void setLengthMotorWithAngleLimits(double output) {
		if (output > 0.0) {
			// Only extend to the max length available in the current angle
			if (this.getCurrentLength() < ArmConstants.maxLengthForAngle(this.getCurrentAngle())) {
				// Set length motors
				this.setLengthMotorWithLimits(output);
			} else {
				this.setLengthMotorWithLimits(0.0);
			}
		} else {
			this.setLengthMotorWithLimits(output);
		}
	}

	/**
	 * @return If both the angle and length motors are at their setpoint.
	 */
	public boolean isAtSetpoint() {
		return this.anglePIDController.atSetpoint() && this.armLengthPIDController.atSetpoint();
	}

	/**
	 * Returns a RunCommand to manually control the arm's length and angle using 3 output suppliers.
	 * 
	 * @param angleOutputSupplier     - The output supplier for the angle motor.
	 * @param forwardsOutputSupplier  - The output supplier for the arm's forwards extension.
	 * @param backwardsOutputSupplier - The output supplier for the arm's backwards retraction.
	 */
	public Command openLoopTeleopArmCommand(DoubleSupplier angleOutputSupplier, DoubleSupplier forwardsOutputSupplier,
			DoubleSupplier backwardsOutputSupplier, PS4Controller controller) {
		return new RunCommand(() -> {
			if (controller.getL3ButtonPressed()) {
				this.shouldOverrideLimits = true;
			} else if (controller.getL3ButtonReleased()) {
				this.shouldOverrideLimits = false;
			}

			// Set angle motor
			double angleSupplierValue = angleOutputSupplier.getAsDouble();
			this.setAngleMotorWithThresholdsAndLimits(angleSupplierValue * angleSupplierValue
					* Math.signum(angleSupplierValue) * ArmConstants.kArmAngleSpeedRatio);

			// Set length motors
			this.setLengthMotorWithThresholdsAndLimits(
					(backwardsOutputSupplier.getAsDouble() - forwardsOutputSupplier.getAsDouble())
							* ArmConstants.kArmLengthSpeedRatio * ArmConstants.kArmLengthSpeedRatio);
		}, this);
	}

	public Command closedLoopTeleopArmCommand(DoubleSupplier angleOutputSupplier, DoubleSupplier forwardsOutputSupplier,
			DoubleSupplier backwardsOutputSupplier) {
		return new RunCommand(() -> {
			// Set angle setpoint. Aim for 1 degree less/more than the limit
			// to avoid jumping (increase the number if it jumps anyway)
			if (angleOutputSupplier.getAsDouble() < 0.0) {
				this.anglePIDController.setSetpoint(ArmConstants.kBottomAngleLimitDeg - 1);
			} else {
				this.anglePIDController.setSetpoint(ArmConstants.kTopAngleLimitDeg + 1);
			}

			this.setAngleMotorWithLengthLimits(this.calculateAngleMotorOutput() * angleOutputSupplier.getAsDouble());

			// Set length setpoint. Aim for 1 degree less/more than the limit
			// to avoid jumping (increase the number if it jumps anyway)
			if (forwardsOutputSupplier.getAsDouble() - backwardsOutputSupplier.getAsDouble() < 0) {
				this.armLengthPIDController.setSetpoint(ArmConstants.kMinArmLengthDeg + 1);
			} else {
				this.armLengthPIDController.setSetpoint(ArmConstants.kMaxArmLengthDeg - 1);
			}

			this.setLengthMotorWithAngleLimits(this.calculateLengthMotorOutput()
					* (forwardsOutputSupplier.getAsDouble() - backwardsOutputSupplier.getAsDouble()));
		}, this); // Require ArmSubsystem
	}

	/**
	 * Returns a command that sets a new state for the command and then runs a closed control loop to get to that
	 * setpoint, and maintains it until interrupted.
	 * 
	 * @param newState - The new state of the arm.
	 */
	public Command setStateCommand(ArmState newState) {
		return new FunctionalCommand(() -> {
			this.setState(newState);
		}, () -> {
			this.setAngleMotorWithLimits(this.calculateAngleMotorOutput());
			if (this.anglePIDController.atSetpoint()) {
				this.setLengthMotorWithLimits(this.calculateLengthMotorOutput());
			}
		}, (interrupted) -> {
			this.setAngleMotorWithLimits(0.0);
			this.setLengthMotorWithLimits(0.0);
		}, () -> {
			return this.isAtSetpoint();
		}, this);
	}

	/**
	 * Returns a command that sets a new state for the command and then runs a closed control loop to get to that
	 * setpoint. Ends the command when it gets to the setpoint if [endAtSetpoint] is true.
	 * 
	 * @param newState      - The new state of the arm.
	 * @param endAtSetpoint - Whether the command should end when it gets to the setpoint or should it maintain the
	 *                      state.
	 */
	public Command setStateCommand(ArmState newState, boolean endAtSetpoint) {
		return new FunctionalCommand(() -> this.setState(newState), () -> {
			this.setAngleMotorWithLimits(this.calculateAngleMotorOutput());
			this.setLengthMotorWithLimits(this.calculateLengthMotorOutput());
		}, (interrupted) -> {
			this.setAngleMotorWithLimits(0.0);
			this.setLengthMotorWithLimits(0.0);
		}, () -> {
			return endAtSetpoint && this.isAtSetpoint();
		}, this);
	}

	/**
	 * Returns a command that sets a new state for the command and then runs a closed control loop to get to that
	 * setpoint. After getting to the new state, it executes the {destinationAction}.
	 * 
	 * @param newState          - The new state of the arm.
	 * @param destinationAction - The command to execute when arriving at the destinationState and before returning to
	 *                          the previous state.
	 */
	public Command setStateWithActionCommand(ArmState newState, Command destinationAction) {
		return new SequentialCommandGroup(this.setStateCommand(newState), destinationAction,
				this.setStateCommand(this.currentState));
	}

	public Command homeCommand() {
		return new FunctionalCommand(() -> {
		}, () -> {
			if (this.getCurrentAngle() > 75.0) {
				this.setAngleMotorWithLimits(0.2);
				return;
			}

			// The limits are normally true
			if (this.retractLimit.get()) {
				this.setLengthMotorWithLimits(0.8);
				this.setAngleMotorWithLimits(0.0);
			} else {
				this.setLengthMotorWithLimits(0.0);
				if (this.bottomAngleLimit.get()) {
					this.setAngleMotorWithLimits(0.2);
				} else {
					this.armAngleMotor.set(0.0);
				}
			}

		}, (interrupted) -> {
			if (!interrupted) {
				this.armLengthCANCoder.setPosition(0.0);
			}
		}, () -> (!this.retractLimit.get() && !this.bottomAngleLimit.get()) || RobotContainer.shoudlArmMove(), this);
	}

	public Command resetLengthCANCoderPositionCommand() {
		return new InstantCommand(() -> this.armLengthCANCoder.setPosition(0.0));
	}

	@Override
	public void periodic() {
		this.extendLimitEntry.setBoolean(!this.extendLimit.get());
		this.retractLimitEntry.setBoolean(!this.retractLimit.get());
		this.topAngleLimitEntry.setBoolean(!this.topAngleLimit.get());
		this.bottomAngleLimitEntry.setBoolean(!this.bottomAngleLimit.get());
	}
}
