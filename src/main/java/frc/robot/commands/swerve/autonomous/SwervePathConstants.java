
package frc.robot.commands.swerve.autonomous;

import java.util.HashMap;
import com.hamosad1657.lib.math.HaUnitConvertor;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

public final class SwervePathConstants {
	/**
	 * The start position of the first auto path. Should match real life because apriltags.
	 */
	public static final Pose2d kStartPose = new Pose2d(0, 0, new Rotation2d());

	public static final double kMaxSpeedMPS = 2.0;
	public static final double kMaxAccelMPSSquared = 3.0;
	public static final double kMaxAngularSpeedRadPS = Math.PI;
	public static final double kMaxAngularAccelRadPSSquared = Math.PI;
	public static final PathConstraints kPathConstraints = new PathConstraints(kMaxSpeedMPS, kMaxAccelMPSSquared);

	// PID gains for path following. If zero it works on only feedforwards.
	/*
	 * How it works: for example, if the proportional gain for X and Y is 1.5, then the controller will add another 1.5
	 * meter/second for every meter of error. If the proportional gain for angle is 2, then the controller will add 2
	 * radians/second for every radian of error.
	 */
	public static final double kXControllerP = 3.5;
	public static final double kXControllerI = 0.0;
	public static final double kXControllerD = 0.0;

	public static final double kYControllerP = kXControllerP;
	public static final double kYControllerI = kXControllerI;
	public static final double kYControllerD = kXControllerD;

	public static final double kAngleControllerP = -10.0;
	public static final double kAngleControllerI = 0.0;
	public static final double kAngleControllerD = 0.0;

	public static final double kPoseToleranceM = 0.05;
	public static final double kAngleToleranceRad = HaUnitConvertor.degToRad(1.0);

	/** X controller. Leaving it 0 will only use feedforwards. */
	public static final PIDController kXController = new PIDController(kXControllerP, kXControllerI, kXControllerD);

	/** Y controller. Leaving it 0 will only use feedforwards. */
	public static final PIDController kYController = new PIDController(kYControllerP, kYControllerI, kYControllerD);

	/** Angle controller. Leaving it 0 will only use feedforwards. */
	public static final PIDController kRotationController = new PIDController(kAngleControllerP, kAngleControllerI,
			kAngleControllerD);

	/* Constraint for the motion profilied robot angle controller */
	public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
			kMaxAngularSpeedRadPS, kMaxAngularAccelRadPSSquared);

	// For use with getPathFollowingCommand
	public static final HashMap<String, Command> kPathCommandsMap = new HashMap<String, Command>();
}
