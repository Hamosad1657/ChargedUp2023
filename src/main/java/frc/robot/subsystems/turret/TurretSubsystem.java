
package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.hamosad1657.lib.sensors.HaCANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
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

	/** Clock-Wise. Normally false. */
	private DigitalInput rotationCWLimitSwitch;
	/** Counter-Clock-Wise. Normally false. */
	private DigitalInput rotationCCWLimitSwitch;

	private TurretSubsystem() {
		this.rotationMotor = new CANSparkMax(RobotMap.kTurretMotorID, MotorType.kBrushless);
		this.rotationMotor.setIdleMode(IdleMode.kBrake);
		this.rotationEncoder = new HaCANCoder(RobotMap.kTurretCANCoderID, TurretConstants.kCANCoderOffsetDeg);
		this.rotationEncoder.setMeasurmentRange(AbsoluteSensorRange.Signed_PlusMinus180);
		this.rotationCCWLimitSwitch = new DigitalInput(RobotMap.kTurretCCWLimitPort);
		this.rotationCWLimitSwitch = new DigitalInput(RobotMap.kTurretCWLimitPort);

		ShuffleboardTab turretTab = Shuffleboard.getTab("Turret");
		turretTab.add("CW Limit", this.rotationCWLimitSwitch).withPosition(0, 0).withSize(1, 1);
		turretTab.add("CCW Limit", this.rotationCCWLimitSwitch).withPosition(0, 1).withSize(1, 1);
		turretTab.add("Rotation CANCoder", this.rotationEncoder).withPosition(1, 0).withSize(2, 2);
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

	/**
	 * @param output - Positive output for CW rotation, Negative output for CCW rotation.
	 */
	public void rotateWithLimits(double output) {
		if ((output < 0.0 && !this.rotationCWLimitSwitch.get())
				|| (output > 0.0 && !this.rotationCCWLimitSwitch.get())) {
			this.setRotationMotor(0.0);
		} else {
			this.setRotationMotor(output * TurretConstants.kRotationMaxOutput);
		}
	}

	public Command openLoopTeleopTurretCommand(DoubleSupplier speedSupplier) {
		return new RunCommand(() -> this.rotateWithLimits(speedSupplier.getAsDouble()), this);
	}
}
