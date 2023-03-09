
package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.hamosad1657.lib.sensors.HaCANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class TurretSubsystem extends SubsystemBase {
	private static TurretSubsystem instace;

	public static TurretSubsystem getInstance() {
		if (instace == null) {
			instace = new TurretSubsystem();
		}
		return instace;
	}

	private CANSparkMax rotationMotor;
	private HaCANCoder rotationEncoder;
	private ShuffleboardTab turretTab;
	private GenericEntry dio0Entry, dio1Entry;

	private PIDController rotationPIDController;

	/** Normally false. */
	private DigitalInput rotationCCWLimitSwitch;
	/** Normally false. */
	private DigitalInput rotationCWLimitSwitch;

	private TurretSubsystem() {
		this.rotationMotor = new CANSparkMax(RobotMap.kTurretMotorID, MotorType.kBrushless);
		this.rotationMotor.setIdleMode(IdleMode.kBrake);
		this.rotationEncoder = new HaCANCoder(RobotMap.kTurretCANCoderID, TurretConstants.kCANCoderOffsetDeg);
		this.rotationEncoder.setMeasurmentRange(AbsoluteSensorRange.Signed_PlusMinus180);
		this.rotationCCWLimitSwitch = new DigitalInput(RobotMap.kTurretCCWLimitPort);
		this.rotationCWLimitSwitch = new DigitalInput(RobotMap.kTurretCWLimitPort);
		this.rotationPIDController = TurretConstants.kRotationPIDGains.toPIDController();
		this.turretTab = Shuffleboard.getTab("Turret");
		this.dio1Entry = this.turretTab.add("CCW Limit", false).withPosition(0, 1).withSize(1, 1).getEntry();
		this.dio0Entry = this.turretTab.add("CW Limit", false).withPosition(0, 0).withSize(1, 1).getEntry();
		this.turretTab.add("Turret Encoder", this.rotationEncoder).withPosition(1, 0).withSize(2, 2);
	}

	/**
	 * @param output - Positive for CW rotation, negative for CCW rotation
	 */
	public void setRotationMotor(double output) {
		this.rotationMotor.set(output);
	}

	public double getAngle() {
		return this.rotationEncoder.getAbsAngleDeg();
	}

	public double calculateRotationMotorOutput() {
		return this.rotationPIDController.calculate(this.getAngle());
	}

	public boolean atAngleSetpoint() {
		return this.rotationPIDController.atSetpoint();
	}

	// The limit switches are normally true (aka true when not pressed,
	// false when pressed)
	/**
	 * @param output - Positive output for CW rotation, Negative output for CCW rotation.
	 */
	public void rotateTurretWithLimits(double output) {
		if ((output < 0.0 && !this.rotationCWLimitSwitch.get())
				|| (output > 0.0 && !this.rotationCCWLimitSwitch.get())) {
			this.setRotationMotor(0.0);
		} else {
			this.setRotationMotor(output * TurretConstants.kSpeedRatio);
		}
	}

	public Command openLoopTeleopTurretCommand(DoubleSupplier speedSupplier) {
		return new RunCommand(() -> this.rotateTurretWithLimits(speedSupplier.getAsDouble()), this);
	}

	public Command closedLoopTeleopTurretCommand(DoubleSupplier output) {
		return new RunCommand(() -> {
			// If negative output, set setpoint to counter-clockwise limit
			if (output.getAsDouble() < 0) {
				this.rotationPIDController.setSetpoint(TurretConstants.kAngleAtCCWLimit);
			}
			// If positive output, set setpoint to clockwise limit
			else {
				this.rotationPIDController.setSetpoint(TurretConstants.kAngleAtCWLimit);
			}
			// Set motor
			this.rotateTurretWithLimits(this.calculateRotationMotorOutput() * output.getAsDouble());
		}, this); // Require this subsystem
	}

	public Command setTurretAngle(double angle) {
		return new FunctionalCommand(() -> this.rotationPIDController.setSetpoint(angle),
				() -> this.rotateTurretWithLimits(this.calculateRotationMotorOutput()),
				(interrupted) -> this.setRotationMotor(0.0), () -> this.atAngleSetpoint());
	}

	@Override
	public void periodic() {
		this.dio0Entry.setBoolean(!this.rotationCCWLimitSwitch.get());
		this.dio1Entry.setBoolean(!this.rotationCWLimitSwitch.get());
	}
}
