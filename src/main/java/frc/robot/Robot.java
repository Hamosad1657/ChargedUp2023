
package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class Robot extends TimedRobot {
	/**
	 * Drivers don't need info from the subsystems in the shuffleboard. To reduce bandwidth, only send neccesary info.
	 */
	public static final boolean showShuffleboardSubsystemInfo = true;

	private RobotContainer robotContainer;
	private CommandScheduler commandScheduler;
	private Command autoCommand;
	private Command testCommand;

	@Override
	public void robotInit() {
		LiveWindow.setEnabled(false);
		CameraServer.startAutomaticCapture().setResolution(80, 60);

		this.robotContainer = new RobotContainer();
		this.commandScheduler = CommandScheduler.getInstance();

		TurretSubsystem.getInstance().setIdleMode(IdleMode.kCoast);
		ArmSubsystem.getInstance().setAngleIdleMode(IdleMode.kCoast);
		ArmSubsystem.getInstance().setLengthIdleMode(IdleMode.kCoast);
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
		if (this.testCommand != null) {
			this.testCommand.cancel();
		}
	}

	@Override
	public void testInit() {
		this.testCommand = this.robotContainer.getAutoCommand();
		if (this.testCommand != null) {
			this.testCommand.schedule();
		}
		
	}

	@Override
	public void disabledInit() {
		SwerveSubsystem.getInstance().crossLockWheels();
	}

	@Override
	public void disabledExit() {
		TurretSubsystem.getInstance().setIdleMode(IdleMode.kBrake);
		ArmSubsystem.getInstance().setAngleIdleMode(IdleMode.kBrake);
		ArmSubsystem.getInstance().setLengthIdleMode(IdleMode.kBrake);
		this.commandScheduler.cancelAll();
	}


	public static void print(Object object) {
		DriverStation.reportWarning(object.toString(), false);
	}

	public static <T> T debug(T object) {
		DriverStation.reportWarning(object.toString(), false);
		return object;
	}
}
