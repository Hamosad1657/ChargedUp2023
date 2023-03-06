
package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.function.BiConsumer;
import com.hamosad1657.lib.math.HaUnits;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.swerve.paths.SwervePathConstants;
import frc.robot.commands.swerve.teleop.TeleopDriveCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class RobotContainer {
	public static final double kJoystickDeadband = 0.075;

	public PS4Controller driverA_Controller, driverB_Controller;
	public CommandPS4Controller driverA_CommandController, driverB_CommandController;

	private final JoystickButton driverA_Share, driverA_R2, driverA_L2, driverA_PS, driverA_Circle, driverA_Cross,
			driverA_Triangle;
	private final JoystickButton driverB_Circle, driverB_Share, driverB_Options;

	private SwerveSubsystem swerve;
	private GrabberSubsystem grabber;
	private ArmSubsystem arm;
	private IntakeSubsystem intake;
	private TurretSubsystem turret;
	private SendableChooser<Command> comboxChooser;

	public RobotContainer() {
		this.driverA_Controller = new PS4Controller(RobotMap.kDriverAControllerUSBPort);
		this.driverB_Controller = new PS4Controller(RobotMap.kDriverBControllerUSBPort);
		this.driverA_CommandController = new CommandPS4Controller(0);
		this.driverB_CommandController = new CommandPS4Controller(1);
		this.arm = ArmSubsystem.getInstance();
		this.grabber = GrabberSubsystem.getInstance();
		this.intake = IntakeSubsystem.getInstance();
		this.turret = TurretSubsystem.getInstance();
		this.swerve = SwerveSubsystem.getInstance();

		this.driverA_Share = new JoystickButton(driverA_Controller, PS4Controller.Button.kShare.value);
		this.driverA_R2 = new JoystickButton(driverA_Controller, PS4Controller.Button.kR2.value);
		this.driverA_L2 = new JoystickButton(driverA_Controller, PS4Controller.Button.kL2.value);
		this.driverA_Circle = new JoystickButton(driverA_Controller, PS4Controller.Button.kCircle.value);
		this.driverA_Cross = new JoystickButton(driverA_Controller, PS4Controller.Button.kCross.value);
		this.driverA_Triangle = new JoystickButton(driverA_Controller, PS4Controller.Button.kTriangle.value);
		this.driverA_PS = new JoystickButton(driverA_Controller, PS4Controller.Button.kPS.value);
		this.driverB_Share = new JoystickButton(driverB_Controller, PS4Controller.Button.kShare.value);
		this.driverB_Options = new JoystickButton(driverB_Controller, PS4Controller.Button.kOptions.value);
		this.driverB_Circle = new JoystickButton(driverB_Controller, PS4Controller.Button.kCircle.value);

		this.configureButtonsBindings();
		this.setDefaultCommands();
		this.createPathsComboBox();
		this.comboxChooser.setDefaultOption("Arm & Mobility", this.swerve.getPathPlannerAutoCommand("Arm & Mobility"));
	}

	private void configureButtonsBindings() {
		this.driverA_Share.onTrue(new InstantCommand(this.swerve::zeroGyro));
		this.driverA_Circle.onTrue(new InstantCommand(() -> this.swerve.resetEstimatedPose(new Pose2d())));
		this.driverA_Circle.onTrue(new InstantCommand(this.swerve::resetOdometry));
		this.driverA_Cross.onTrue(this.swerve.crossLockWheelsCommand());
		this.driverA_Triangle.onTrue(new InstantCommand(this.swerve::toggleSwerveSpeed));
		this.driverA_PS.onTrue(new InstantCommand(this.swerve::modulesToZero, this.swerve));

		this.driverB_Circle.onTrue(this.grabber.toggleGrabberSolenoidCommand());

		this.driverB_Share.onTrue(this.arm.homeCommand());
		this.driverB_Options.onTrue(this.arm.resetLengthCANCoderPositionCommand());

		this.driverA_R2.onTrue(this.intake.lowerIntakeCommand());
		this.driverA_L2.onTrue(this.intake.raiseIntakeCommand());

		this.driverA_CommandController.share().onTrue(new InstantCommand(this.swerve::zeroGyro));
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
		this.arm.setDefaultCommand(this.arm.openLoopTeleopArmCommand(
				() -> HaUnits.deadband(driverB_Controller.getLeftY(), kJoystickDeadband),
				() -> HaUnits.deadband((driverB_Controller.getR2Axis() + 1.0), kJoystickDeadband),
				() -> HaUnits.deadband((driverB_Controller.getL2Axis() + 1.0), kJoystickDeadband), driverB_Controller));

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

	public static boolean shoudlArmMove() {
		double lengthValue = (driverB_Controller.getL2Axis() + 1.0) - (driverB_Controller.getR2Axis() + 1.0);
		double angleValue = driverB_Controller.getLeftY();

		return (lengthValue > kJoystickDeadband || lengthValue < -kJoystickDeadband || angleValue > kJoystickDeadband
				|| angleValue < -kJoystickDeadband);
	}

	/**
	 * Creats and adds the widget for selecting the paths for autonomous.
	 */
	private void createPathsComboBox() {
		ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
		this.comboxChooser = new SendableChooser<Command>();
		SwervePathConstants.kPaths.forEach(new BiConsumer<String, Command>() {
			@Override
			public void accept(String name, Command command) {
				comboxChooser.addOption(name, command);
			}
		});
		autoTab.add("Path Chooser", this.comboxChooser).withWidget("ComboBox Chooser");
	}
}
