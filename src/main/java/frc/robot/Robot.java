
package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {
	private Command autonomousCommand;
	private RobotContainer robotContainer;

	/**
	 * This function is run when the robot is first started up and should be used for any initialization code.
	 */
	@Override
	public void robotInit() {
		CameraServer.startAutomaticCapture().setResolution(80, 60);

		// Instantiate our RobotContainer. This will perform all our button bindings,
		// and put our
		// autonomous chooser on the dashboard.
		this.robotContainer = new RobotContainer();
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for items like diagnostics that you want
	 * ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard integrated
	 * updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
	}

	/**
	 * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		this.autonomousCommand = this.robotContainer.getAutoCommand();
		if (this.autonomousCommand != null) {
			this.autonomousCommand.schedule();
		}
	}

	@Override
	public void teleopInit() {
		// Tell the robot that it's facing us
		SwerveSubsystem.getInstance().zeroGyroWith(180.0);

		if (this.autonomousCommand != null) {
			this.autonomousCommand.cancel();
		}
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
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
