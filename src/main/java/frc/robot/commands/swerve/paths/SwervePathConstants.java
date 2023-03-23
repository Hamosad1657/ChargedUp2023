
package frc.robot.commands.swerve.paths;

import java.util.HashMap;
import com.hamosad1657.lib.math.HaUnitConvertor;
import com.hamosad1657.lib.math.HaUnits;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.ArmConstants.ArmState;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeConstants.ShootHeight;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretSubsystem;

public final class SwervePathConstants {
	/**
	 * The start position of the first auto path. Should match real life because
	 * apriltags.
	 */
	public static final Pose2d kStartPose = new Pose2d(0, 0, new Rotation2d());

	public static final double kMaxSpeedMPS = 2.0;
	public static final double kMaxAccelMPSSquared = 3.0;
	public static final double kMaxAngularSpeedRadPS = Math.PI;
	public static final double kMaxAngularAccelRadPSSquared = Math.PI;
	public static final PathConstraints kPathConstraints = new PathConstraints(kMaxSpeedMPS, kMaxAccelMPSSquared);

	// PID gains for path following. If zero it works on only feedforwards.
	/*
	 * How it works: for example, if the proportional gain for X and Y is 1.5, then
	 * the controller will add another 1.5
	 * meter/second for every meter of error. If the proportional gain for angle is
	 * 2, then the controller will add 2
	 * radians/second for every radian of error.
	 */

	public static final HaUnits.PIDGains kXControllerGains = new HaUnits.PIDGains(8.0, 0.0, 0.0);
	public static final HaUnits.PIDGains kYControllerGains = kXControllerGains;
	public static final HaUnits.PIDGains kRotationControllerGains = new HaUnits.PIDGains(-10.0, 0.0, 0.0);

	public static final double kPoseToleranceM = 0.05;
	public static final double kAngleToleranceRad = HaUnitConvertor.degToRad(1.0);

	/** X controller. Leaving it 0 will only use feedforwards. */
	public static final PIDController kXController = kXControllerGains.toPIDController();

	/** Y controller. Leaving it 0 will only use feedforwards. */
	public static final PIDController kYController = kYControllerGains.toPIDController();

	/** Angle controller. Leaving it 0 will only use feedforwards. */
	public static final PIDController kRotationController = kRotationControllerGains.toPIDController();

	/* Constraint for the motion profilied robot angle controller */
	public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
			kMaxAngularSpeedRadPS, kMaxAngularAccelRadPSSquared);

	/**
	 * Command to be used in PathPlanner paths + their names, are added to this map
	 * in
	 * {@link SwervePathConstants#createPathCommands()}. The names in the
	 * PathPlanner app must match the names in the
	 * code.
	 * <p>
	 * this map is passed as a parameter when constructing the auto builder in
	 * SwerveSubsystem.
	 */
	public static final HashMap<String, Command> kPathCommandsMap = new HashMap<String, Command>();

	/**
	 * A HashMap of command groups and their names, to choose from in the drop-down
	 * menue in the shuffleboard and then
	 * run in autonomous.
	 * <p>
	 * Options are added to this map using SwerveSubsystem.addPath(), which is
	 * called as
	 * many times as needed in SwerveSubsystem.createPaths().
	 */
	public static final HashMap<String, SequentialCommandGroup> kAutoOptionsMap = new HashMap<String, SequentialCommandGroup>();

	/** Add command options to use in PathPlanner paths. */
	public static void createPathCommands() {
		ArmSubsystem arm = ArmSubsystem.getInstance();
		GrabberSubsystem grabber = GrabberSubsystem.getInstance();
		IntakeSubsystem intake = IntakeSubsystem.getInstance();
		TurretSubsystem turret = TurretSubsystem.getInstance();

		// Wait a bit
		SwervePathConstants.kPathCommandsMap.put("WaitABitLess", new WaitCommand(0.10));
		SwervePathConstants.kPathCommandsMap.put("WaitABit", new WaitCommand(0.15));
		SwervePathConstants.kPathCommandsMap.put("WaitABitMore", new WaitCommand(0.35));
		SwervePathConstants.kPathCommandsMap.put("WaitABitMore++", new WaitCommand(0.5));
		SwervePathConstants.kPathCommandsMap.put("WaitABitMore#", new WaitCommand(0.75));
		SwervePathConstants.kPathCommandsMap.put("WaitABitMoreJava", new WaitCommand(1.0));

		// Turret
		SwervePathConstants.kPathCommandsMap.put("RotateTurretBack",
				turret.getToSetpointWithHomingCommand(TurretConstants.kBackRotationSetpoint));
		SwervePathConstants.kPathCommandsMap.put("RotateTurretFront",
				turret.getToSetpointWithHomingCommand(TurretConstants.kFrontRotationSetpoint));
		SwervePathConstants.kPathCommandsMap.put("FlipTurret",
				turret.flipTurretCommand().deadlineWith(arm.getToStateCommand(ArmState.kHalfClosed, true)));

		// Arm
		SwervePathConstants.kPathCommandsMap.put("HomeArm", arm.autoHomeCommand().withTimeout(3.0));
		SwervePathConstants.kPathCommandsMap.put("ArmHigh", arm.getToStateCommand(ArmState.kHigh, true));
		SwervePathConstants.kPathCommandsMap.put("ArmMid", arm.getToStateCommand(ArmState.kMid, true));
		SwervePathConstants.kPathCommandsMap.put("ArmLowCone", arm.getToStateCommand(ArmState.kLowCone, true));
		SwervePathConstants.kPathCommandsMap.put("ArmLowCube", arm.getToStateCommand(ArmState.kLowCube, true));
		SwervePathConstants.kPathCommandsMap.put("ArmHalfClosed",
				arm.getToStateLengthFirstCommand(ArmState.kHalfClosed, true).withTimeout(4.0)
						.handleInterrupt(() -> arm.autoHomeCommand().schedule()));
		SwervePathConstants.kPathCommandsMap.put("RetractArm", arm.retractCommand().withTimeout(4.0));
		SwervePathConstants.kPathCommandsMap.put("AlmostHome", arm.getToStateCommand(ArmState.kAlmostHome, true));

		// Grabber
		SwervePathConstants.kPathCommandsMap.put("CollectGamePiece", grabber.collectCommand());
		SwervePathConstants.kPathCommandsMap.put("ReleaseGamePiece", grabber.releaseCommand());

		// Intake
		SwervePathConstants.kPathCommandsMap.put("OpenIntake",
				intake.lowerCommand().withTimeout(1.0));
		SwervePathConstants.kPathCommandsMap.put("CloseIntake", intake.autoRaiseCommand().withTimeout(0.75));
		SwervePathConstants.kPathCommandsMap.put("GetIntakeToAngle",
				intake.getToShootHeightCommand(ShootHeight.kAuto).withTimeout(0.75));
		SwervePathConstants.kPathCommandsMap.put("ShootGamePiece", intake.shootCommand(true).withTimeout(1.0));
		SwervePathConstants.kPathCommandsMap.put("CollectGamePiece", intake.autoCollectPieceCommand().withTimeout(1.5));

		// Pickups
		SwervePathConstants.kPathCommandsMap.put("PickupCone", arm.pickupConeCommand());
		SwervePathConstants.kPathCommandsMap.put("PickupCube",
				grabber.collectCommand().andThen(arm.getToStateCommand(ArmState.kLowCube, true)));

		// Dropoffs
		SwervePathConstants.kPathCommandsMap.put("DropoffLowCone",
				arm.getToStateCommand(ArmState.kLowConeDropoff, true).andThen(grabber.releaseCommand()));
		SwervePathConstants.kPathCommandsMap.put("DropoffLowCube",
				arm.getToStateCommand(ArmState.kLowCube, true).andThen(grabber.releaseCommand()));
		SwervePathConstants.kPathCommandsMap.put("DropoffMid",
				arm.getToStateCommand(ArmState.kMid, true).andThen(grabber.releaseCommand()));
		SwervePathConstants.kPathCommandsMap.put("DropoffHigh",
				arm.getToStateCommand(ArmState.kHigh, true).withTimeout(8.0).andThen(grabber.releaseCommand()));

		// Turret & Pickups
		SwervePathConstants.kPathCommandsMap.put("FlipTurretPickupCube",
				turret.flipTurretCommand().deadlineWith(arm.getToStateCommand(ArmState.kHalfClosed, true))
						.andThen(arm.getToStateCommand(ArmState.kLowCube, true).andThen(grabber.collectCommand())));

	}
}
