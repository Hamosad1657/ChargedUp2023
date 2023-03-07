
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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmConstants.ArmState;
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
	private final PIDController armLengthPIDController;
	private final ProfiledPIDController anglePIDController;
	private final DigitalInput extendLimit, retractLimit, bottomAngleLimit, topAngleLimit;
	private final GenericEntry extendLimitEntry, retractLimitEntry, topAngleLimitEntry, bottomAngleLimitEntry,
			angleMotorOutputEntry;
	private ArmState currentState;
	private int currentStateIndex;
	private boolean shouldOverrideLimits = false;

	// Angle lowers when you go up
	public ArmSubsystem() {
		this.armAngleMotor = new CANSparkMax(RobotMap.kArmAngleMotorID, MotorType.kBrushless);
		this.armAngleMotor.setIdleMode(IdleMode.kBrake);
		this.armAngleMotor.setInverted(true);
		this.armAngleMotor.enableVoltageCompensation(12.0);
		this.armAngleCANCoder = new HaCANCoder(RobotMap.kArmAngleCANCoderID, ArmConstants.kAngleCANCoderOffsetDeg);
		this.armAngleCANCoder.setMeasurmentRange(AbsoluteSensorRange.Unsigned_0_to_360);
		this.armAngleCANCoder.setPosition(0.0);

		this.armLengthMotorA = new WPI_TalonSRX(RobotMap.kArmLengthMotorAID);
		this.armLengthMotorA.setNeutralMode(NeutralMode.Brake);
		this.armLengthMotorB = new WPI_TalonSRX(RobotMap.kArmLengthMotorBID);
		this.armLengthMotorB.setNeutralMode(NeutralMode.Brake);
		this.armLengthMotorB.follow(this.armLengthMotorA);

		this.armLengthMotor = new HaTalonSRX(armLengthMotorA);
		this.armLengthCANCoder = new HaCANCoder(RobotMap.kArmLengthCANCoderID, ArmConstants.kArmLengthEncoderOffset);
		this.armLengthCANCoder.setMeasurmentRange(AbsoluteSensorRange.Unsigned_0_to_360);
		this.armLengthCANCoder.setPosition(0);

		this.anglePIDController = new ProfiledPIDController(ArmConstants.kAngleP, ArmConstants.kAngleI,
				ArmConstants.kAngleD, ArmConstants.kAngleTrapezoidProfile,
				ArmConstants.kAngleControllerUpdateWaitPeriod);
		this.anglePIDController.setTolerance(ArmConstants.kArmAngleTolerance);

		this.armLengthPIDController = ArmConstants.kArmLengthPIDGains.toPIDController();
		this.armLengthPIDController.setTolerance(ArmConstants.kArmLengthTolerance);

		this.extendLimit = new DigitalInput(RobotMap.kArmExtendLimitPort);
		this.retractLimit = new DigitalInput(RobotMap.kArmRetractLimitPort);
		this.bottomAngleLimit = new DigitalInput(RobotMap.kBottomArmAngleLimitport);
		this.topAngleLimit = new DigitalInput(RobotMap.kTopArmAngleLimitport);

		this.currentStateIndex = 0;

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

	public void resetLengthCANCoder() {
		this.armLengthCANCoder.setPosition(0.0);
	}

	private double getCurrentLength() {
		return -this.armLengthCANCoder.getPositionDeg();
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
		this.anglePIDController.setGoal(newState.angleDeg);
		this.armLengthPIDController.setSetpoint(newState.lengthDeg);
	}

	public void moveArmStateUp() {
		if (this.currentStateIndex < ArmConstants.kArmStates.length) {
			this.currentStateIndex++;
			this.currentState = ArmConstants.kArmStates[this.currentStateIndex];
		}
	}

	public void moveArmStateDown() {
		if (this.currentStateIndex > 0) {
			this.currentStateIndex--;
			this.currentState = ArmConstants.kArmStates[this.currentStateIndex];
		}
	}

	/**
	 * @return The output for the angle motor calculated by the PID in [-1.0, 1.0].
	 */
	public double calculateAngleMotorOutput() {
		double output = -MathUtil.clamp(this.anglePIDController.calculate(this.getCurrentAngle()),
				-ArmConstants.kAngleMotorMaxSpeed, ArmConstants.kAngleMotorMaxSpeed);
		return output;
	}

	/**
	 * @return The output for the length motor calculated by the PID in [-1.0, 1.0].
	 */
	public double calculateLengthMotorOutput() {
		double output = -MathUtil.clamp(this.armLengthPIDController.calculate(this.getCurrentLength()),
				-ArmConstants.kLengthMotorMaxOutput, ArmConstants.kLengthMotorMaxOutput);
		return output;
	}

	/**
	 * @return If both the angle and length motors are at their setpoint.
	 */
	public boolean isAtSetpoint() {
		return this.anglePIDController.atGoal() && this.armLengthPIDController.atSetpoint();
	}

	public Command getToStateCommand() {
		return new RunCommand(() -> {
			this.setAngleMotorWithLimits(this.calculateAngleMotorOutput());
			this.setLengthMotorWithLimits(this.calculateLengthMotorOutput());
		}, this);
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
	 *               Doesn't slow near the limits.
	 */
	public void setLengthMotorWithLimits(double output) {
		// The magnetic limit switches are normally true.
		if (!this.shouldOverrideLimits && output < 0 && !this.extendLimit.get()) {
			this.armLengthMotor.set(0.0);
		} else if (!this.shouldOverrideLimits && output > 0 && !this.retractLimit.get()) {
			this.armLengthMotor.set(0.0);
		} else {
			this.armLengthMotor.set(output);
		}
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
			if (controller.getL1ButtonPressed()) {
				this.shouldOverrideLimits = true;
			} else if (controller.getL1ButtonReleased()) {
				this.shouldOverrideLimits = false;
			}

			// Set angle motor
			double angleSupplierValue = angleOutputSupplier.getAsDouble();
			this.setAngleMotorWithLimits(angleSupplierValue * angleSupplierValue * Math.signum(angleSupplierValue)
					* ArmConstants.kAngleMotorMaxSpeed);

			// Set length motors
			this.setLengthMotorWithLimits((backwardsOutputSupplier.getAsDouble() - forwardsOutputSupplier.getAsDouble())
					* ArmConstants.kArmLengthSpeedRatio * ArmConstants.kArmLengthSpeedRatio);
		}, this);
	}

	public Command homeCommand() {
		return new FunctionalCommand(() -> {
		}, () -> {
			if (this.getCurrentAngle() > 70.0) {
				this.setAngleMotorWithLimits(0.25);
				return;
			}

			// The limits are normally true
			if (this.getCurrentLength() > 150) {
				this.setLengthMotorWithLimits(0.85);
				this.setAngleMotorWithLimits(0.0);
			} else {
				this.setLengthMotorWithLimits(0.0);
				if (this.bottomAngleLimit.get()) {
					this.setAngleMotorWithLimits(0.25);
				} else {
					this.armAngleMotor.set(0.0);
				}
			}

		}, (interrupted) -> {
			if (!interrupted) {
				this.armLengthCANCoder.setPosition(0.0);
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

	@Override
	public void periodic() {
		this.extendLimitEntry.setBoolean(!this.extendLimit.get());
		this.retractLimitEntry.setBoolean(!this.retractLimit.get());
		this.topAngleLimitEntry.setBoolean(!this.topAngleLimit.get());
		this.bottomAngleLimitEntry.setBoolean(!this.bottomAngleLimit.get());
	}
}
