
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
import frc.robot.subsystems.arm.ArmConstants.ArmState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretSubsystem;

public class RobotContainer {
	public static final double kJoystickDeadband = 0.075;

	public static PS4Controller driverA_Controller, driverB_Controller;
	public CommandPS4Controller driverA_CommandController, driverB_CommandController;

	private ArmSubsystem arm;
	private GrabberSubsystem grabber;
	private IntakeSubsystem intake;
	private SwerveSubsystem swerve;
	private TurretSubsystem turret;
	private SendableChooser<Command> comboBoxChooser;

	public RobotContainer() {
		driverA_Controller = new PS4Controller(RobotMap.kDriverA_ControllerUSBPort);
		driverB_Controller = new PS4Controller(RobotMap.kDriverB_ControllerUSBPort);
		this.driverA_CommandController = new CommandPS4Controller(RobotMap.kDriverA_ControllerUSBPort);
		this.driverB_CommandController = new CommandPS4Controller(RobotMap.kDriverB_ControllerUSBPort);

		this.arm = ArmSubsystem.getInstance();
		this.grabber = GrabberSubsystem.getInstance();
		this.intake = IntakeSubsystem.getInstance();
		this.swerve = SwerveSubsystem.getInstance();
		this.turret = TurretSubsystem.getInstance();

		this.configureButtonsBindings();
		this.setDefaultCommands();
		this.createPathsComboBox();
	}

	private void configureButtonsBindings() {
		// Swerve
		this.driverA_CommandController.share().onTrue(new InstantCommand(this.swerve::zeroGyro));
		this.driverA_CommandController.cross().onTrue(this.swerve.crossLockWheelsCommand());
		this.driverA_CommandController.triangle().onTrue(new InstantCommand(this.swerve::toggleSwerveSpeed));

		// Intake
		this.driverA_CommandController.R2().onTrue(this.intake.lowerIntakeCommand());
		this.driverA_CommandController.L2().onTrue(this.intake.raiseIntakeCommand());

		// Arm
		this.driverB_CommandController.povUp().onTrue(this.arm.getToStateCommand(ArmState.kHigh));
		this.driverB_CommandController.povLeft().onTrue(this.arm.getToStateCommand(ArmState.kMid));
		this.driverB_CommandController.povRight().onTrue(this.arm.pickupConeCommand());
		this.driverB_CommandController.povDown().onTrue(this.arm.getToStateCommand(ArmState.kLowCube));
		this.driverB_CommandController.options()
				.onTrue(this.grabber.collectCommand().alongWith(this.arm.getToStateCommand(ArmState.kShelf)));
		this.driverB_CommandController.square().onTrue(this.arm.getToStateCommand(ArmState.kLowRaiseCone));
		this.driverB_CommandController.circle().onTrue(this.arm.getToStateCommand(ArmState.kLowConeDropoff));
		this.driverB_CommandController.share().onTrue(this.arm.homeCommand());

		// Grabber
		this.driverB_CommandController.cross().onTrue(this.grabber.collectCommand());
		this.driverB_CommandController.triangle().onTrue(this.grabber.releaseCommand());

		// Turret
		this.driverB_CommandController.R1()
				.onTrue(new InstantCommand(() -> this.turret.setSetpoint(TurretConstants.kFrontRotationSetpoint)));
		this.driverB_CommandController.L1()
				.onTrue(new InstantCommand(() -> this.turret.setSetpoint(TurretConstants.kBackRotationSetpoint)));
	}

	private void setDefaultCommands() {
		// All of the actions are detailed in the DRIVING_INSTRUCTIONS.md file.

		// Swerve teleop driving - Left stick for X and Y movement, right X for rotation.
		this.swerve.setDefaultCommand(new TeleopDriveCommand(this.swerve,
				() -> HaUnits.deadband(-driverA_Controller.getLeftX(), kJoystickDeadband),
				() -> HaUnits.deadband(driverA_Controller.getLeftY(), kJoystickDeadband),

				() -> HaUnits.deadband(driverA_Controller.getRightX(), kJoystickDeadband)));

		// Turret teleop control - Right X for rotation.
		this.turret.setDefaultCommand(this.turret
				.closedLoopTeleopCommand(() -> HaUnits.deadband(driverB_Controller.getRightX(), kJoystickDeadband)));

		// Teleop arm control - R2 for extending, L2 for retracting, left Y for angle.
		this.arm.setDefaultCommand(this.arm.closedLoopTeleopCommand(
				() -> HaUnits.deadband(driverB_Controller.getLeftY(), kJoystickDeadband),
				() -> HaUnits.deadband((driverB_Controller.getR2Axis() + 1.0), kJoystickDeadband),
				() -> HaUnits.deadband((driverB_Controller.getL2Axis() + 1.0), kJoystickDeadband)));

		// Intake keep up - Not teleop
		this.intake.setDefaultCommand(this.intake.keepIntakeUpCommand());
	}

	/**
	 * Use this to pass the auto command to Robot.java.
	 * 
	 * @return The command to run in autonomous
	 */
	public Command getAutoCommand() {
		return this.comboBoxChooser.getSelected();
	}

	/**
	 * Adds the widget for selecting an autonomous path.
	 */
	private void createPathsComboBox() {
		ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
		this.comboBoxChooser = new SendableChooser<Command>();

		SwervePathConstants.kPaths.forEach((name, command) -> comboBoxChooser.addOption(name, command));
		autoTab.add("Path Chooser", this.comboBoxChooser).withWidget("ComboBox Chooser");

		this.comboBoxChooser.setDefaultOption("Arm & Mobility",
				this.swerve.getPathPlannerAutoCommand("Arm & Mobility"));
	}
}
