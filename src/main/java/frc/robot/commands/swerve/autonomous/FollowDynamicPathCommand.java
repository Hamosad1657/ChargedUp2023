
package frc.robot.commands.swerve.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveConstants.Auto;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * A class for generating a short path on-the-fly in order to get to a point, then following said path. For more complex
 * paths, use FollowJSONPathCommand.
 */
public class FollowDynamicPathCommand extends CommandBase {
	private SwerveSubsystem swerve;
	private PathPlannerTrajectory trajectory;
	private PPHolonomicDriveController driveController;
	private PathConstraints pathConstraints;
	private Pose2d endPose;
	private Timer timer;

	public FollowDynamicPathCommand(SwerveSubsystem swerve, Pose2d endPose, PathConstraints pathConstraints,
			double poseToleranceM, double angleToleranceRad) {
		this.swerve = swerve;
		this.addRequirements(this.swerve);

		this.pathConstraints = pathConstraints;
		this.endPose = endPose;

		this.driveController = new PPHolonomicDriveController(Auto.kXController, Auto.kYController,
				Auto.kAngleController);
		this.driveController
				.setTolerance(new Pose2d(poseToleranceM, poseToleranceM, Rotation2d.fromRadians(angleToleranceRad)));
		this.driveController.setEnabled(true);

		this.timer = new Timer();
	}

	public FollowDynamicPathCommand(SwerveSubsystem swerve, Pose2d endPose) {
		this(swerve, endPose, Auto.kPathConstraints, Auto.kPoseToleranceM, Auto.kAngleToleranceRad);
	}

	@Override
	public void initialize() {
		Pose2d startPose = this.swerve.getEstimatedPose();

		/*
		 * Heading is the angle of the point from which you arrive or leave, Holonomic rotation is the robot's angle
		 * when it arrives / leaves the point.
		 */
		Rotation2d heading = calculateHeadingBetweenPoints(startPose, endPose);

		Translation2d startPointTranslation = startPose.getTranslation();
		Rotation2d startPointRobotRotation = startPose.getRotation();
		PathPoint pathStartPoint = new PathPoint(startPointTranslation, heading, startPointRobotRotation);

		Translation2d endPointTranslation = this.endPose.getTranslation();
		Rotation2d endPointRobotRotation = this.endPose.getRotation();
		PathPoint pathEndPoint = new PathPoint(endPointTranslation, heading, endPointRobotRotation);

		this.trajectory = PathPlanner.generatePath(this.pathConstraints, pathStartPoint, pathEndPoint);
		Robot.print("Successfully generated dynamic path: " + startPose + " -> " + this.endPose);

		this.timer.reset();
		this.timer.start();
		this.swerve.setRobotAngleCorrection(false);
	}

	@Override
	public void execute() {
		// Trajectories contain states (velocities accelerations and positions) for any
		// given time.
		// Get the desired state by sampling the path for the current time.
		double currentTime = this.timer.get();
		PathPlannerState desiredState = (PathPlannerState) this.trajectory.sample(currentTime);

		// Calculate the desired field-relative ChassisSpeeds using the current position
		// and desired state.
		Pose2d currentPose = this.swerve.getEstimatedPose();
		ChassisSpeeds fieldRelativeSpeeds = this.driveController.calculate(currentPose, desiredState);

		// Drive
		this.swerve.autonomousDrive(fieldRelativeSpeeds, false, true);
	}

	@Override
	public boolean isFinished() {
		// End the command if the path's duration has passed, AND the robot is
		// within it's position tolerance.
		double pathDurationSeconds = this.trajectory.getTotalTimeSeconds();
		boolean hasPathDurationPassed = this.timer.hasElapsed(pathDurationSeconds);
		return hasPathDurationPassed && this.driveController.atReference();
	}

	@Override
	public void end(boolean interrupted) {
		Robot.print("Finished dynamic path");
	}

	/**
	 * @return The heading of pose1 towards pose2.
	 */
	private static Rotation2d calculateHeadingBetweenPoints(Pose2d pose1, Pose2d pose2) {
		Pose2d pose1RelativeToPose2 = pose1.relativeTo(pose2);
		return new Rotation2d(-pose1RelativeToPose2.getX(), -pose1RelativeToPose2.getY());
	}
}
