
package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Robot extends TimedRobot {
	/**
	 * Drivers don't need info from the subsystems in the shuffleboard. To reduce bandwidth, only send neccesary info.
	 */
	public static final boolean showShuffleboardSubsystemInfo = true;

	private RobotContainer robotContainer;
	private CommandScheduler commandScheduler;
	private Command autoCommand;

	@Override
	public void robotInit() {
		CameraServer.startAutomaticCapture().setResolution(80, 60);

		this.robotContainer = new RobotContainer();
		this.commandScheduler = CommandScheduler.getInstance();
	}

	@Override
	public void robotPeriodic() {
		this.commandScheduler.run();
	}

	@Override
	public void autonomousInit() {
		this.autoCommand = this.robotContainer.getAutoCommand();
		if (this.autoCommand != null) {
			this.autoCommand.schedule();
		}
	}

	@Override
	public void teleopInit() {
		if (this.autoCommand != null) {
			this.autoCommand.cancel();
		}
	}

	@Override
	public void disabledInit() {
		SwerveSubsystem.getInstance().crossLockWheels();
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	public static void print(Object object) {
		DriverStation.reportWarning(object.toString(), false);
	}

	public static <T> T debug(T object) {
		DriverStation.reportWarning(object.toString(), false);
		return object;
	}
}
