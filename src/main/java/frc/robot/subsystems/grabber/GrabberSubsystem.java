
package frc.robot.subsystems.grabber;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
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

	private GrabberSubsystem() {
		this.grabberSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotMap.kGrabberSolenoidPort);
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
}
