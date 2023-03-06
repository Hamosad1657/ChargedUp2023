
package frc.robot.subsystems.grabber;

import com.hamosad1657.lib.motors.HaCANSparkMax;
import com.hamosad1657.lib.sensors.HaColorSensor;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class GrabberSubsystem extends SubsystemBase {
	private static GrabberSubsystem instance;

	public static GrabberSubsystem getInstance() {
		if (instance == null) {
			instance = new GrabberSubsystem();
		}
		return instance;
	}

	private final HaCANSparkMax grabberMotor;
	private final HaColorSensor colorSensor;

	private GrabberSubsystem() {
		this.colorSensor = new HaColorSensor(RobotMap.kColorSensorPort);
		CANSparkMax motor = new CANSparkMax(RobotMap.kGrabberMotorID, MotorType.kBrushless);
		this.grabberMotor = new HaCANSparkMax(motor);

		ShuffleboardTab tab = Shuffleboard.getTab("Arm");
		tab.add("Color Sensor", this.colorSensor).withPosition(0, 2).withSize(3, 3);
		tab.addBoolean("Game Piece In Grabber", this::isGamePieceInRange);
		tab.add("Grabber Motor", this.grabberMotor);
	}

	/**
	 * Releases the game piece.
	 */
	public void releaseGamePiece() {
		this.grabberMotor.set(-GrabberConstants.kCollectGamePieceSpeed);
	}

	/**
	 * Releases the game piece.
	 */
	public Command releaseGamePieceCommand() {
		return new InstantCommand(this::releaseGamePiece, this);
	}

	/**
	 * Check if there's either a cone or a cube in the grabber's range.
	 */
	private boolean isGamePieceInRange() {
		return this.isCubeInRange() || this.isConeInRange();
	}

	/**
	 * Check if there's a cube in the grabber's range.
	 */
	private boolean isCubeInRange() {
		return this.colorSensor.isColorInRangePercent(GrabberConstants.kCubeMinColor, GrabberConstants.kCubeMaxColor)
				&& this.colorSensor.isObjectInProximityRange(GrabberConstants.kCubeMinDistance,
						GrabberConstants.kCubeMaxDistance);
	}

	/**
	 * Check if there's a cone in the grabber's range.
	 */
	private boolean isConeInRange() {
		return this.colorSensor.isColorInRangePercent(GrabberConstants.kConeMinColor, GrabberConstants.kConeMaxColor)
				&& this.colorSensor.isObjectInProximityRange(GrabberConstants.kConeMinDistance,
						GrabberConstants.kConeMaxDistance);
	}

	/**
	 * Collects the game piece.
	 */
	private void collectGamePiece() {
		this.grabberMotor.set(GrabberConstants.kCollectGamePieceSpeed);
	}

	/**
	 * Collects the game piece.
	 */
	public Command collectGamePieceCommand() {
		return new InstantCommand(this::collectGamePiece, this);
	}

	@Override
	public void periodic() {
		if (isConeInRange())
			this.grabberMotor.motor.setSmartCurrentLimit(GrabberConstants.kMaxCubeAmper);
		if (isConeInRange())
			this.grabberMotor.motor.setSmartCurrentLimit(GrabberConstants.kMaxConeAmper);

	}

}
