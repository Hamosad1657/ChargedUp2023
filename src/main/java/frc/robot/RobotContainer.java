
package frc.robot;

import com.hamosad1657.lib.math.HaUnits;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.commands.swerve.paths.SwervePathConstants;
import frc.robot.commands.swerve.teleop.TeleopDriveCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmConstants.ArmState;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class RobotContainer {
	public static final double kJoystickDeadband = 0.075;

	public static PS4Controller driverA_Controller, driverB_Controller;
	public CommandPS4Controller driverA_CommandController, driverB_CommandController;

	private SwerveSubsystem swerve;
	private GrabberSubsystem grabber;
	private ArmSubsystem arm;
	private IntakeSubsystem intake;
	private TurretSubsystem turret;
	private SendableChooser<Command> comboxChooser;

	public RobotContainer() {
		driverA_Controller = new PS4Controller(RobotMap.kDriverA_ControllerUSBPort);
		driverB_Controller = new PS4Controller(RobotMap.kDriverB_ControllerUSBPort);
		this.driverA_CommandController = new CommandPS4Controller(RobotMap.kDriverA_ControllerUSBPort);
		this.driverB_CommandController = new CommandPS4Controller(RobotMap.kDriverB_ControllerUSBPort);

		this.arm = ArmSubsystem.getInstance();
		this.grabber = GrabberSubsystem.getInstance();
		this.intake = IntakeSubsystem.getInstance();
		this.turret = TurretSubsystem.getInstance();
		this.swerve = SwerveSubsystem.getInstance();

		this.configureButtonsBindings();
		this.setDefaultCommands();
		this.createPathsComboBox();
	}

	private void configureButtonsBindings() {
		this.driverA_CommandController.share().onTrue(new InstantCommand(this.swerve::zeroGyro));
		this.driverA_CommandController.cross().onTrue(this.swerve.crossLockWheelsCommand());
		this.driverA_CommandController.triangle().onTrue(new InstantCommand(this.swerve::toggleSwerveSpeed));

		this.driverA_CommandController.R2().onTrue(this.intake.lowerIntakeCommand());
		this.driverA_CommandController.L2().onTrue(this.intake.raiseIntakeCommand());

		this.driverB_CommandController.povUp().onTrue(this.arm.getToStateCommand(ArmState.kHigh));
		this.driverB_CommandController.povLeft().onTrue(this.arm.getToStateCommand(ArmState.kMid));
		this.driverB_CommandController.povRight().onTrue(this.arm.getToStateCommand(ArmState.kLowCone));
		this.driverB_CommandController.povDown().onTrue(this.arm.getToStateCommand(ArmState.kLowCube));
		this.driverB_CommandController.options().onTrue(this.arm.getToStateCommand(ArmState.kShelf));
		this.driverB_CommandController.share().onTrue(this.arm.homeCommand());

		this.driverB_CommandController.cross().onTrue(this.grabber.collectCommand());
		this.driverB_CommandController.triangle().onTrue(this.grabber.releaseCommand());

		this.driverB_CommandController.PS().onTrue(new InstantCommand(this.arm::resetLengthCANCoder));
	}

	private void setDefaultCommands() {
		// All of the actions are detailed in the DRIVING_INSTRUCTIONS.md file.

		// Swerve teleop drive command
		this.swerve.setDefaultCommand(new TeleopDriveCommand(this.swerve, driverA_Controller::getLeftY,
				driverA_Controller::getLeftX, driverA_Controller::getRightX));

		// Teleop turret command - right X
		this.turret.setDefaultCommand(this.turret.openLoopTeleopTurretCommand(driverB_Controller::getRightX));
		// this.turret.setDefaultCommand(this.turret.closedLoopTeleopTurretCommand(driverB_Controller::getRightX));

		// Teleop arm open/close - R2 open, L2 close, left Y for angle
		this.arm.setDefaultCommand(this.arm.closedLoopTeleopCommand(
				() -> HaUnits.deadband(driverB_Controller.getLeftY(), kJoystickDeadband),
				() -> HaUnits.deadband((driverB_Controller.getR2Axis() + 1.0), kJoystickDeadband),
				() -> HaUnits.deadband((driverB_Controller.getL2Axis() + 1.0), kJoystickDeadband)));
		// this.arm.setDefaultCommand(this.arm.openLoopTeleopCommand(
		// () -> HaUnits.deadband(driverB_Controller.getLeftY(), kJoystickDeadband),
		// () -> HaUnits.deadband((driverB_Controller.getR2Axis() + 1.0), kJoystickDeadband),
		// () -> HaUnits.deadband((driverB_Controller.getL2Axis() + 1.0), kJoystickDeadband)));

		// Keep intake up
		this.intake.setDefaultCommand(this.intake.keepIntakeUpCommand());
	}

	/**
	 * Use this to pass the auto command to Robot.java.
	 * 
	 * @return The command to run in autonomous
	 */
	public Command getAutoCommand() {
		return this.comboxChooser.getSelected();
	}

	/**
	 * Creats and adds the widget for selecting the paths for autonomous.
	 */
	private void createPathsComboBox() {
		ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
		this.comboxChooser = new SendableChooser<Command>();

		SwervePathConstants.kPaths.forEach((name, command) -> comboxChooser.addOption(name, command));
		autoTab.add("Path Chooser", this.comboxChooser).withWidget("ComboBox Chooser");

		this.comboxChooser.setDefaultOption("Arm & Mobility", this.swerve.getPathPlannerAutoCommand("Arm & Mobility"));
	}
}
