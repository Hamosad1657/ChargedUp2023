
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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;
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

		this.rotationEncoder = new HaCANCoder(RobotMap.kTurretCANCoderID, TurretConstants.kCANCoderOffsetDeg);
		this.rotationEncoder.setMeasurmentRange(AbsoluteSensorRange.Unsigned_0_to_360);

		this.rotationController = TurretConstants.kRotationPIDGains.toPIDController();
		this.rotationController.setTolerance(TurretConstants.kRotationTolerance);
		this.rotationController.disableContinuousInput();
		this.resetSetpoint();

		this.rotationCCWLimitSwitch = new DigitalInput(RobotMap.kTurretCCWLimitPort);
		this.rotationCWLimitSwitch = new DigitalInput(RobotMap.kTurretCWLimitPort);

		if (Robot.showShuffleboardSubsystemInfo) {
			ShuffleboardTab turretTab = Shuffleboard.getTab("Turret");
			turretTab.addBoolean("CW Limit", () -> !this.rotationCWLimitSwitch.get()).withPosition(0, 0).withSize(1, 1);
			turretTab.addBoolean("CCW Limit", () -> !this.rotationCCWLimitSwitch.get()).withPosition(1, 0).withSize(1,
					1);
			turretTab.add("Rotation CANCoder", this.rotationEncoder).withPosition(0, 1).withSize(2, 2);
			turretTab.addDouble("Rotation", this.rotationEncoder::getAbsAngleDeg).withWidget("Gyro").withPosition(2, 1)
					.withSize(2, 2);
			turretTab.addDouble("Rotation Setpoint", this.rotationController::getSetpoint).withWidget("Gyro")
					.withPosition(4, 1).withSize(2, 2);
			turretTab.addBoolean("At Rotation Setpoint", this::isAtSetpoint).withSize(2, 1).withPosition(2, 0);
		}
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

	public boolean isAtSetpoint(double tolerance) {
		return Math.abs(this.rotationController.getPositionError()) < tolerance;
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

	public void resetSetpoint() {
		this.setSetpoint(this.getCurrentAngle());
	}

	public Command getToSetpointCommand(double rotation) {
		return new InstantCommand(() -> this.setSetpoint(rotation))
				.andThen(new WaitUntilCommand(() -> this.isAtSetpoint(TurretConstants.kAutoRotationTolerance)));
	}

	public Command getToSetpointWithHomingCommand(double rotation) {
		return new InstantCommand(() -> this.setSetpoint(rotation))
				.andThen(new WaitUntilCommand(() -> this.isAtSetpoint(TurretConstants.kAutoRotationTolerance)))
				.deadlineWith(ArmSubsystem.getInstance().autoHomeCommand());
	}

	public Command flipTurretCommand() {
		return new InstantCommand(() -> {
			double setpoint;

			// Within a 10 degree tolerance to 90
			if (Math.abs(TurretConstants.kFrontRotationSetpoint - this.getCurrentAngle()) < 10.0) {
				setpoint = TurretConstants.kBackRotationSetpoint;
			} else {
				setpoint = TurretConstants.kFrontRotationSetpoint;
			}

			this.setSetpoint(setpoint);
		}).andThen(new WaitUntilCommand(() -> this.isAtSetpoint(TurretConstants.kAutoRotationTolerance)));
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

	public void setIdleMode(IdleMode idleMode) {
		this.rotationMotor.setIdleMode(idleMode);
	}
}
