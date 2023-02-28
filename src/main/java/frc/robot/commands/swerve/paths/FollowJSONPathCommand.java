
package frc.robot.commands.swerve.paths;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

/**
 * Generates a PathPlanner path from a JSON (with or without events) and follows it.
 */
public class FollowJSONPathCommand extends CommandBase {
	private SwerveSubsystem swerve;
	private PathPlannerTrajectory trajectory;
	private PPHolonomicDriveController driveController;
	private List<EventMarker> eventMarkersList;
	private Pose2d positionTolerance;
	private int currentEventIndex = 0;
	private Command[] pathCommandsList;
	private Timer timer;
	private String pathName;

	/**
	 * @param swerve
	 * @param pathName          - The path's name as specified in PathPlanner.
	 * @param constraints       - Max velocity in MPS, and max accel in MPS^2.
	 * @param poseToleranceM
	 * @param angleToleranceRad
	 * @param pathCommandsList  - An array containing Commands to run during path event markers, indexed by the order
	 *                          they should run.
	 */
	public FollowJSONPathCommand(SwerveSubsystem swerve, String pathName, Command[] pathCommandsList,
			PathConstraints constraints, double poseToleranceM, double angleToleranceRad) {
		this.swerve = swerve;
		this.addRequirements(this.swerve);

		this.pathName = pathName;
		this.trajectory = PathPlanner.loadPath(this.pathName, constraints);
		Robot.print("Sucsessfully loaded path from JSON.");

		this.positionTolerance = new Pose2d(poseToleranceM, poseToleranceM, Rotation2d.fromRadians(angleToleranceRad));
		// Controls the robot's pose. When disabled it only uses feedforward; when
		// enabled it also uses three PID controllers to fix the position error.
		this.driveController = new PPHolonomicDriveController(SwervePathConstants.kXController,
				SwervePathConstants.kYController, SwervePathConstants.kRotationController);
		this.driveController.setTolerance(this.positionTolerance);
		this.driveController.setEnabled(true);

		this.eventMarkersList = this.trajectory.getMarkers();
		this.pathCommandsList = pathCommandsList;

		this.timer = new Timer();
	}

	public FollowJSONPathCommand(SwerveSubsystem swerve, String pathName, Command[] pathCommandsList) {
		this(swerve, pathName, pathCommandsList, SwervePathConstants.kPathConstraints,
				SwervePathConstants.kPoseToleranceM, SwervePathConstants.kAngleToleranceRad);
	}

	@Override
	public void initialize() {
		this.timer.reset();
		this.timer.start();
		this.swerve.setRobotAngleCorrection(false);

		Pose2d initialPose = this.trajectory.getInitialHolonomicPose();
		this.swerve.resetOdometry(initialPose);
		this.swerve.setGyro(initialPose.getRotation());
		this.swerve.resetEstimatedPose(initialPose);
	}

	@Override
	public void execute() {
		// Trajectories contain states (velocities accelerations and positions) for any
		// given time.
		// Get the desired state by sampling the path for the current time.
		PathPlannerState desiredState = (PathPlannerState) this.trajectory.sample(this.timer.get());

		// Calculate the desired field-relative ChassisSpeeds using the current position
		// and desired state.
		ChassisSpeeds fieldRelativeSpeeds = this.driveController.calculate(this.swerve.getEstimatedPose(),
				desiredState);

		// Drive
		this.swerve.autonomousDrive(fieldRelativeSpeeds, false, true);

		// If there are event markers in the path, and the robot is within the position
		// tolerance
		// for one of the event markers, schedule the command with the same index, and
		// increase the
		// index by 1.
		if (!this.eventMarkersList.isEmpty() && this.currentEventIndex < this.eventMarkersList.size()) {
			if (this.swerve.withinPositionTolerance(this.eventMarkersList.get(currentEventIndex).positionMeters,
					this.positionTolerance)) {
				this.pathCommandsList[currentEventIndex].schedule();
				Robot.print("Scheduled the command " + this.pathCommandsList[currentEventIndex].getName());
				this.currentEventIndex++;
			}
		}
	}

	@Override
	public boolean isFinished() {
		// End the command if the time the path takes has passed, AND the robot is
		// within it's position tolerance.
		return this.timer.hasElapsed(this.trajectory.getTotalTimeSeconds()) && this.driveController.atReference();
	}

	@Override
	public void end(boolean interrupted) {
		Robot.print("Finished path: " + this.pathName);
	}
}
