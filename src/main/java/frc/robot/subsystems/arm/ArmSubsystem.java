
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmConstants.ArmState;
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
	private final PIDController armLengthPIDController, armAnglePIDController;
	private final ArmFeedforward armAngleFFController;
	private final DigitalInput extendLimit, retractLimit, bottomAngleLimit, topAngleLimit;
	private final GenericEntry extendLimitEntry, retractLimitEntry, topAngleLimitEntry, bottomAngleLimitEntry,
			angleMotorOutputEntry;
	private ArmState currentState;

	// Angle lowers when you go up
	public ArmSubsystem() {
		this.armAngleMotor = new CANSparkMax(RobotMap.kArmAngleMotorID, MotorType.kBrushless);
		this.armAngleMotor.setIdleMode(IdleMode.kBrake);
		this.armAngleMotor.enableVoltageCompensation(12.0);
		this.armAngleCANCoder = new HaCANCoder(RobotMap.kArmAngleCANCoderID, ArmConstants.kAngleCANCoderOffsetDeg);
		this.armAngleCANCoder.setMeasurmentRange(AbsoluteSensorRange.Signed_PlusMinus180);

		this.armLengthMotorA = new WPI_TalonSRX(RobotMap.kArmLengthMotorAID);
		this.armLengthMotorA.setNeutralMode(NeutralMode.Brake);
		this.armLengthMotorB = new WPI_TalonSRX(RobotMap.kArmLengthMotorBID);
		this.armLengthMotorB.setNeutralMode(NeutralMode.Brake);
		this.armLengthMotorB.follow(this.armLengthMotorA);

		this.armLengthMotor = new HaTalonSRX(armLengthMotorA);
		this.armLengthCANCoder = new HaCANCoder(RobotMap.kArmLengthCANCoderID, ArmConstants.kArmLengthEncoderOffset);

		this.armAnglePIDController = ArmConstants.kArmAnglePIDGains.toPIDController();
		this.armAnglePIDController.setTolerance(ArmConstants.kArmAngleTolerance);
		// TODO: Implement arm angle FF control
		this.armAngleFFController = new ArmFeedforward(ArmConstants.kArmAngleFFkS, ArmConstants.kArmAngleFFkG,
				ArmConstants.kArmAngleFFkV);

		this.armLengthPIDController = ArmConstants.kArmLengthPIDGains.toPIDController();
		this.armLengthPIDController.setTolerance(ArmConstants.kArmLengthTolerance);

		this.extendLimit = new DigitalInput(RobotMap.kArmExtendLimitPort);
		this.retractLimit = new DigitalInput(RobotMap.kArmRetractLimitPort);
		this.bottomAngleLimit = new DigitalInput(RobotMap.kBottomArmAngleLimitport);
		this.topAngleLimit = new DigitalInput(RobotMap.kTopArmAngleLimitport);

		this.currentState = ArmState.kDefaultState;

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

	/**
	 * Sets the setpoint of the arm and length motors to the new arm state.
	 * 
	 * @param newState - The new arm state.
	 */
	public void setState(ArmState newState) {
		this.armAnglePIDController.setSetpoint(newState.angleDeg);
		this.armLengthPIDController.setSetpoint(newState.getPulleyDistance());
	}

	/**
	 * @return The output for the angle motor calculated by the PID in [-1.0, 1.0].
	 */
	public double calculateAngleMotorOutput() {
		double PIDOutput = this.armAnglePIDController.calculate(this.armAngleCANCoder.getAbsAngleDeg());

		// Used in this way, the FF controller will only help maintain a position, not reach a setpoint.
		double FFOutput = this.armAngleFFController.calculate(this.armAngleCANCoder.getAbsAngleRad(), 0.0);

		return MathUtil.clamp(PIDOutput + FFOutput, -ArmConstants.kMaxMotorOutput, ArmConstants.kMaxMotorOutput);

		// TODO: After tuning the PID gains, switch the code above to this code:
		// return
		// this.armAnglePIDController.calculate(this.armAngleCANCoder.getAbsAngleDeg());
	}

	/**
	 * @return The output for the length motor calculated by the PID in [-1.0, 1.0].
	 */
	public double calculateLengthMotorOutput() {
		return MathUtil.clamp(this.armLengthPIDController.calculate(this.armLengthCANCoder.getAbsAngleDeg()),
				-ArmConstants.kMaxMotorOutput, ArmConstants.kMaxMotorOutput);

		// TODO: After tuning the PID gains, switch the code above to this code:
		// return this.armLengthPIDController.calculate(this.armLengthCANCoder.getAngleDeg());
	}

	/**
	 * @param output - The output of the angle motor in [-1.0, 1.0]. Positive output: arm goes down, negative output:
	 *               arm goes up.
	 */
	public void setAngleMotorWithLimits(double output) {
		// The magnetic limit switches are normally true.
		if ((output < 0.0 && !this.topAngleLimit.get()) || (output > 0.0 && !this.bottomAngleLimit.get())) {
			this.angleMotorOutputEntry.setDouble(0.0);
			this.armAngleMotor.set(0.0);
		} else {
			if ((output > -ArmConstants.kArmAngleBalanceMotorOutput
					&& output < ArmConstants.kArmAngleBalanceMotorOutput)) {
				if (this.armAngleCANCoder.getAbsAngleDeg() > -85.0) {
					output = 0.0;
				} else {
					output = -ArmConstants.kArmAngleBalanceMotorOutput;
				}
			}
			this.angleMotorOutputEntry.setDouble(output);
			this.armAngleMotor.set(output);
		}
	}

	/**
	 * @param output - The output of the length motor in [-1.0, 1.0].
	 */
	public void setLengthMotorWithLimits(double output) {
		if (output < 0 && !this.extendLimit.get()) { // The magnetic limit switches are normally true.
			this.armLengthMotor.set(0.0);
		} else if (output > 0 && !this.retractLimit.get()) {
			this.armLengthMotor.set(0.0);
		} else {
			this.armLengthMotor.set(output * ArmConstants.kArmLengthSpeedRatio);
		}
	}

	/**
	 * @return If both the angle and length motors are at their setpoint.
	 */
	public boolean isAtSetpoint() {
		return this.armAnglePIDController.atSetpoint() && this.armLengthPIDController.atSetpoint();
	}

	/**
	 * Returns a RunCommand to manually control the arm's length and angle using 3 output suppliers.
	 * 
	 * @param angleOutputSupplier     - The output supplier for the angle motor.
	 * @param forwardsOutputSupplier  - The output supplier for the arm's forwards extension.
	 * @param backwardsOutputSupplier - The output supplier for the arm's backwards retraction.
	 */
	public Command openLoopTeleopArmCommand(DoubleSupplier angleOutputSupplier, DoubleSupplier forwardsOutputSupplier,
			DoubleSupplier backwardsOutputSupplier) {
		return new RunCommand(() -> {
			// Set angle motor
			double angleSupplierValue = angleOutputSupplier.getAsDouble();
			this.setAngleMotorWithLimits(angleSupplierValue * angleSupplierValue * Math.signum(angleSupplierValue)
					* ArmConstants.kArmAngleSpeedRatio);

			// Set length motors
			this.setLengthMotorWithLimits((forwardsOutputSupplier.getAsDouble() - backwardsOutputSupplier.getAsDouble())
					* ArmConstants.kArmLengthSpeedRatio);
		}, this);
	}

	public Command closedLoopTeleopArmCommand(DoubleSupplier angleOutputSupplier, DoubleSupplier forwardsOutputSupplier,
			DoubleSupplier backwardsOutputSupplier) {
		return new RunCommand(() -> {
			// Set angle setpoint. Aim for 1 degree less/more than the limit
			// to avoid jumping (increase the number if it jumps anyway)
			if (angleOutputSupplier.getAsDouble() < 0.0) {
				this.armAnglePIDController.setSetpoint(ArmConstants.kBottomAngleLimitDeg - 1);
			} else {
				this.armAnglePIDController.setSetpoint(ArmConstants.kTopAngleLimitDeg + 1);
			}

			this.setAngleMotorWithLimits(this.calculateAngleMotorOutput() * angleOutputSupplier.getAsDouble());

			// Set length setpoint. Aim for 1 degree less/more than the limit
			// to avoid jumping (increase the number if it jumps anyway)
			if (forwardsOutputSupplier.getAsDouble() - backwardsOutputSupplier.getAsDouble() < 0) {
				this.armLengthPIDController.setSetpoint(ArmConstants.kMinLength + 1);
			} else {
				this.armLengthPIDController.setSetpoint(ArmConstants.kMaxLength - 1);
			}

			// Set length motors
			this.setLengthMotorWithLimits(this.calculateAngleMotorOutput()
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
		return new FunctionalCommand(() -> this.setState(newState), () -> {
			this.setAngleMotorWithLimits(this.calculateAngleMotorOutput());
			this.setLengthMotorWithLimits(this.calculateLengthMotorOutput());
		}, (interrupted) -> {
			this.setAngleMotorWithLimits(0.0);
			this.setLengthMotorWithLimits(0.0);
		}, () -> {
			return false;
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

	@Override
	public void periodic() {
		this.extendLimitEntry.setBoolean(!this.extendLimit.get());
		this.retractLimitEntry.setBoolean(!this.retractLimit.get());
		this.topAngleLimitEntry.setBoolean(!this.topAngleLimit.get());
		this.bottomAngleLimitEntry.setBoolean(!this.bottomAngleLimit.get());
	}

}