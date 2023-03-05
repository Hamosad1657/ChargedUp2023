
package frc.robot.subsystems.grabber;

import com.hamosad1657.lib.sensors.HaColorSensor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
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

	private final Solenoid grabberSolenoid;
	private final HaColorSensor colorSensor;

	private GrabberSubsystem() {
		this.colorSensor = new HaColorSensor(RobotMap.kColorSensorPort);
		this.grabberSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.kGrabberSolenoidPort);

		ShuffleboardTab tab = Shuffleboard.getTab("Arm");
		tab.addBoolean("Game Piece In Grabber", this::isGamePieceInRange);
		tab.add("Color Sensor", this.colorSensor).withPosition(0, 2).withSize(3, 3);
	}

	/**
	 * Set the value of the intake's left solenoid.
	 * 
	 * @param on - True will turn the solenoid output on. False will turn the solenoid output off.
	 */
	public void setGrabberSolenoid(boolean on) {
		this.grabberSolenoid.set(on);
	}

	/**
	 * Toggles the grabber solenoid.
	 */
	public void toggleGrabberSolenoid() {
		this.grabberSolenoid.toggle();
	}

	/**
	 * Toggles the grabber solenoid.
	 */
	public Command toggleGrabberSolenoidCommand() {
		return new InstantCommand(() -> this.grabberSolenoid.toggle(), this);
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
}
