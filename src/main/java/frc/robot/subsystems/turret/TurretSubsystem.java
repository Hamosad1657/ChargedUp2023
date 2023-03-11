
package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.hamosad1657.lib.sensors.HaCANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.subsystems.arm.ArmSubsystem;

public class TurretSubsystem extends SubsystemBase {
	private static TurretSubsystem instace;

	public static TurretSubsystem getInstance() {
		if (instace == null) {
			instace = new TurretSubsystem();
		}
		return instace;
	}

	private final CANSparkMax rotationMotor;
	private final HaCANCoder rotationEncoder;
	private final PIDController rotationController;

	/** Clock-Wise. Normally false. */
	private final DigitalInput rotationCWLimitSwitch;
	/** Counter-Clock-Wise. Normally false. */
	private final DigitalInput rotationCCWLimitSwitch;

	private TurretSubsystem() {
		this.rotationMotor = new CANSparkMax(RobotMap.kTurretMotorID, MotorType.kBrushless);
		this.rotationMotor.setIdleMode(IdleMode.kBrake);

		this.rotationEncoder = new HaCANCoder(RobotMap.kTurretCANCoderID, TurretConstants.kCANCoderOffsetDeg);
		this.rotationEncoder.setMeasurmentRange(AbsoluteSensorRange.Unsigned_0_to_360);

		this.rotationController = TurretConstants.kRotationPIDGains.toPIDController();
		this.rotationController.setSetpoint(this.getCurrentAngle());
		this.rotationController.setTolerance(TurretConstants.kRotationTolerance);

		this.rotationCCWLimitSwitch = new DigitalInput(RobotMap.kTurretCCWLimitPort);
		this.rotationCWLimitSwitch = new DigitalInput(RobotMap.kTurretCWLimitPort);

		ShuffleboardTab turretTab = Shuffleboard.getTab("Turret");
		turretTab.addBoolean("CW Limit", () -> !this.rotationCWLimitSwitch.get()).withPosition(0, 0).withSize(1, 1);
		turretTab.addBoolean("CCW Limit", () -> !this.rotationCCWLimitSwitch.get()).withPosition(1, 0).withSize(1, 1);
		turretTab.add("Rotation CANCoder", this.rotationEncoder).withPosition(0, 1).withSize(2, 2);
		turretTab.addDouble("Rotation", this.rotationEncoder::getAbsAngleDeg).withWidget("Gyro").withPosition(2, 0)
				.withSize(2, 2);
		turretTab.addDouble("Rotation Setpoint", this.rotationController::getSetpoint).withWidget("Gyro")
				.withPosition(4, 0).withSize(2, 2);
		turretTab.addBoolean("At Rotation Setpoint", this::isAtSetpoint).withSize(2, 1).withPosition(2, 2);
	}

	/**
	 * @param output - Positive for CW rotation, negative for CCW rotation
	 */
	public void setRotationMotor(double output) {
		this.rotationMotor.set(output);
	}

	public double getCurrentAngle() {
		return this.rotationEncoder.getAbsAngleDeg();
	}

	public boolean isAtSetpoint() {
		return this.rotationController.atSetpoint();
	}

	public double calculateRotationMotorOutput() {
		double output = this.rotationController.calculate(this.getCurrentAngle());
		return MathUtil.clamp(output, -TurretConstants.kMotorMaxOutput, TurretConstants.kMotorMaxOutput);
	}

	private boolean isAtCWLimit() {
		return !this.rotationCWLimitSwitch.get() || this.getCurrentAngle() < TurretConstants.kRotationMinAngle;
	}

	private boolean isAtCCWLimit() {
		return !this.rotationCCWLimitSwitch.get() || this.getCurrentAngle() > TurretConstants.kRotationMaxAngle;
	}

	/**
	 * @param output - Positive output for CW rotation, Negative output for CCW rotation.
	 */
	public void rotateWithLimits(double output) {
		if ((output < 0.0 && this.isAtCWLimit()) || (output > 0.0 && this.isAtCCWLimit())) {
			this.setRotationMotor(0.0);
		} else {
			this.setRotationMotor(output);
		}
	}

	public void setSetpoint(double rotation) {
		this.rotationController.setSetpoint(rotation);
	}

	public Command setSetpointCommand(double rotation) {
		return new InstantCommand(() -> this.setSetpoint(rotation))
				.andThen(ArmSubsystem.getInstance().homeCommand().until(this::isAtSetpoint));
	}

	public Command openLoopTeleopCommand(DoubleSupplier outputSupplier) {
		return new RunCommand(() -> {
			this.rotateWithLimits(outputSupplier.getAsDouble() * TurretConstants.kMotorMaxOutput);
		}, this);
	}

	public Command closedLoopTeleopCommand(DoubleSupplier outputSupplier) {
		return new RunCommand(() -> {
			double newSetpoint = this.rotationController.getSetpoint()
					+ outputSupplier.getAsDouble() * TurretConstants.kRotationTeleopSetpointMultiplier;
			this.rotationController.setSetpoint(
					MathUtil.clamp(newSetpoint, TurretConstants.kRotationMinAngle, TurretConstants.kRotationMaxAngle));

			this.rotationMotor.set(this.calculateRotationMotorOutput());
		}, this);
	}
}
