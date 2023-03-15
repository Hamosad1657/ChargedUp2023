
package frc.robot.subsystems.swerve;

import java.util.ArrayList;
import java.util.List;
import com.hamosad1657.lib.sensors.HaNavX;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fusionLib.swerve.SwerveModule;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.commands.swerve.chargestation.BalanceChassisCommand;
import frc.robot.commands.swerve.paths.SwervePathConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;

public class SwerveSubsystem extends SubsystemBase {
	private static SwerveSubsystem instance;

	/** Get the singleton instance of this subsystem. */
	public static SwerveSubsystem getInstance() {
		if (instance == null) {
			instance = new SwerveSubsystem();
		}
		return instance;
	}

	/**
	 * An array of 4 SwerveModule objects (by 364), ordered:
	 * <ul>
	 * <li>Front-left
	 * <li>Front-right
	 * <li>Back-left
	 * <li>Back-right
	 * </ul>
	 */
	private final SwerveModule[] modules;
	private final HaNavX gyro;
	private final SwerveDriveOdometry odometry;
	private final SlewRateLimiter speedModeRateLimiter;
	private final SwerveAutoBuilder autoBuilder;

	/** A PIDController for fixing the robot angle skew. */
	private final PIDController anglePIDController;
	private final Timer angleControlTimer;

	private Field2d field;

	private double teleopAngleSetpointRad;
	private boolean isRobotAngleCorrectionEnabled = false, runAngleCorrection = false;
	public double currentSwerveTranslateRatio = SwerveConstants.kSwerveTranslateRatioFast,
			filteredTranslationRatio = SwerveConstants.kSwerveTranslateRatioFast,
			currentSwerveRotationRatio = SwerveConstants.kSwerveRotationRatioFast;

	private SwerveSubsystem() {
		this.modules = new SwerveModule[] {
				new SwerveModule(0, SwerveConstants.FrontLeftModule.constants),
				new SwerveModule(1, SwerveConstants.FrontRightModule.constants),
				new SwerveModule(2, SwerveConstants.BackLeftModule.constants),
				new SwerveModule(3, SwerveConstants.BackRightModule.constants)
		};
		this.gyro = new HaNavX(RobotMap.kNavXPort);
		this.odometry = new SwerveDriveOdometry(SwerveConstants.kSwerveKinematics, this.getYaw(),
				this.getModulesPositions(), SwervePathConstants.kStartPose);
		this.speedModeRateLimiter = new SlewRateLimiter(SwerveConstants.kSpeedModeRateLimit);

		// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to
		// create an auto command.
		this.autoBuilder = new SwerveAutoBuilder(
				// Pose2d supplier
				this::getOdometryPose,
				// Pose2d consumer, used to reset odometry at the beginning of auto
				this::resetOdometry, SwerveConstants.kSwerveKinematics,
				// PID constants to correct for translation and rotation error
				SwervePathConstants.kXControllerGains.toPathPlannerPIDConstants(),
				SwervePathConstants.kRotationControllerGains.toPathPlannerPIDConstants(),
				// Module states consumer used to output to the drive subsystem
				this::setModuleStates, SwervePathConstants.kPathCommandsMap,
				// Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
				true,
				// The drive subsystem. Used to properly set the requirements of path following commands
				this);

		this.anglePIDController = SwerveConstants.kRobotAnglePIDGains.toPIDController();
		this.anglePIDController.enableContinuousInput(-Math.PI, Math.PI);
		this.anglePIDController.setTolerance(SwerveConstants.kRobotAngleToleranceRad);
		this.angleControlTimer = new Timer();
		this.angleControlTimer.start();

		if (Robot.showShuffleboardSubsystemInfo) {
			ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
			ShuffleboardTab odometryTab = Shuffleboard.getTab("Odometry");

			swerveTab.add("Gyro", this.gyro).withSize(2, 3).withPosition(0, 4);
			swerveTab.add("Angle PID", this.anglePIDController);

			swerveTab.add("Front Left Module", this.modules[0]).withSize(2, 2).withPosition(0, 0);
			swerveTab.add("Front Right Module", this.modules[1]).withSize(2, 2).withPosition(2, 0);
			swerveTab.add("Back Left Module", this.modules[2]).withSize(2, 2).withPosition(4, 0);
			swerveTab.add("Back Right Module", this.modules[3]).withSize(2, 2).withPosition(6, 0);

			odometryTab.addDouble("Odometry X", () -> this.getOdometryPose().getX()).withSize(1, 1).withPosition(0, 0);
			odometryTab.addDouble("Odometry Y", () -> this.getOdometryPose().getY()).withSize(1, 1).withPosition(1, 0);

			this.field = new Field2d();
			odometryTab.add("Field", this.field).withSize(5, 4).withPosition(0, 1);
		}

		SwervePathConstants.createCommands();
		this.createPaths();

		/*
		 * By pausing init for a second before setting module offsets, we avoid a bug with inverting motors. See
		 * https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
		 */
		Timer.delay(0.1);
		this.resetModulesToAbsolute();
	}

	/**
	 * Drive the swerve.
	 * 
	 * @param translation     - A field-relative or robot-relative direction+speed to move in.
	 * @param rotationRadPS   - A rotation speed in RadPS. Open-loop
	 * @param isRobotRelative - Whether the passed Translation2d is field-relative or robot-relative (true if robot
	 *                        relative).
	 * @param isOpenLoop      - Is the wheel speed controlled open-loop or closed-loop.
	 */
	public void teleopDrive(Translation2d translation, double rotationRadPS, boolean isRobotRelative,
			boolean isOpenLoop) {
		// Set the swerveModuleStates from the desired ChassisSpeeds using kinematics.
		SwerveModuleState[] swerveModuleStates = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
				// If robot relative, just convert to ChassisSpeeds.
				// If field relative, convert to robot-relative ChassisSpeeds with gyro.
				isRobotRelative
						? new ChassisSpeeds(translation.getX(), translation.getY(),
								this.calculateAngleCorrectionRadPS(rotationRadPS))
						: ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(),
								this.calculateAngleCorrectionRadPS(rotationRadPS), this.getYaw()));

		// If any of the speeds are above the maximum, lower them all in the same ratio.
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.kChassisMaxSpeedMPS);

		// Set all the module states.
		for (SwerveModule module : this.modules) {
			module.setState(swerveModuleStates[module.moduleNumber], isOpenLoop);
		}
	}

	public void autonomousDrive(ChassisSpeeds chassisSpeeds, boolean isRobotRelativeSpeeds, boolean isOpenLoop) {
		SwerveModuleState[] swerveModuleStates = SwerveConstants.kSwerveKinematics
				.toSwerveModuleStates(isRobotRelativeSpeeds
						? ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds.vxMetersPerSecond,
								chassisSpeeds.vyMetersPerSecond,
								this.calculateAngleCorrectionRadPS(chassisSpeeds.omegaRadiansPerSecond), this.getYaw())
						: new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
								this.calculateAngleCorrectionRadPS(chassisSpeeds.omegaRadiansPerSecond)));

		// If any of the speeds are above the maximum, lower them all in the same ratio.
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.kChassisMaxSpeedMPS);

		// Set all the module states.
		for (SwerveModule module : this.modules) {
			module.setState(swerveModuleStates[module.moduleNumber], isOpenLoop);
		}
	}

	public void toggleSwerveSpeed() {
		if (this.currentSwerveTranslateRatio == SwerveConstants.kSwerveTranslateRatioFast) {
			this.currentSwerveTranslateRatio = SwerveConstants.kSwerveTranslateRatioSlow;
			this.currentSwerveRotationRatio = SwerveConstants.kSwerveRotationRatioSlow;
		} else {
			this.currentSwerveTranslateRatio = SwerveConstants.kSwerveTranslateRatioFast;
			this.currentSwerveRotationRatio = SwerveConstants.kSwerveRotationRatioFast;
		}
	}

	/** Enable or disable angle correction. */
	public void setRobotAngleCorrection(boolean enabled) {
		this.isRobotAngleCorrectionEnabled = enabled;
	}

	/** Set the modules to angle zero to help with finding offsets. */
	public void modulesToZero() {
		for (SwerveModule module : this.modules) {
			module.setState(new SwerveModuleState(), true);
		}
	}

	/* Used by SwerveControllerCommand in Auto */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kChassisMaxSpeedMPS);

		for (SwerveModule module : this.modules) {
			module.setState(desiredStates[module.moduleNumber], false);
		}
	}

	/**
	 * @return The robot position as measured by the odometry. Units in meters and Rotation2d.
	 */
	public Pose2d getOdometryPose() {
		return this.odometry.getPoseMeters();
	}

	/** Set the odometry to a specific position. Units in meters and Rotation2d. */
	public void resetOdometry(Pose2d pose) {
		this.setGyro(pose.getRotation().getDegrees() + 90.0);
		Robot.print("Gyro reset to: " + Double.toString(this.getYaw().getDegrees()));
		this.odometry.resetPosition(this.getYaw(), this.getModulesPositions(), pose);
		Robot.print("Odometry reset to: " + Double.toString(this.getOdometryPose().getX()) + " | "
				+ Double.toString(this.getOdometryPose().getY()));
	}

	/**
	 * Set the odometry to the origin point and facing away from the driver. Units in meters and Rotation2d.
	 */
	public void resetOdometry() {
		this.resetOdometry(new Pose2d());
	}

	/**
	 * @return An array of the current module states, which are speed in MPS and angle in Rotation2d. Ordered
	 *         front-left, front-right, back-left, back-right.
	 */
	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (SwerveModule mod : modules) {
			states[mod.moduleNumber] = mod.getModuleState();
		}
		return states;
	}

	/**
	 * @return An array of the current module positions, which are distance in meters and angle in Rotation2d. Ordered
	 *         front-left, front-right, back-left, back-right.
	 */
	public SwerveModulePosition[] getModulesPositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (SwerveModule mod : modules) {
			positions[mod.moduleNumber] = mod.getModulePosition();
		}
		return positions;
	}

	/**
	 * Sets the yaw angle as 0. This can be used to set the angle the robot is currently facing as "forwards".
	 */
	public void zeroGyro() {
		this.gyro.zeroYaw(180.0);
		this.teleopAngleSetpointRad = Math.PI;
	}

	public void setGyro(double offsetDeg) {
		this.gyro.zeroYaw(offsetDeg);
		this.teleopAngleSetpointRad = this.getYaw().getRadians();
	}

	public void setGyro(Rotation2d offset) {
		this.gyro.zeroYaw(offset);
		this.teleopAngleSetpointRad = this.getYaw().getRadians();
	}

	/**
	 * Returns the angle measured by the navX, inverted to adhere to WPILib's axis conventions.
	 */
	public Rotation2d getYaw() {
		return (SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - this.gyro.getYawAngleDeg())
				: Rotation2d.fromDegrees(this.gyro.getYawAngleDeg());
	}

	/**
	 * Returns the angle measured by the navX, inverted to adhere to WPILib's axis conventions.
	 */
	public Rotation2d getPitch() {
		return (SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - this.gyro.getPitchAngleDeg())
				: Rotation2d.fromDegrees(this.gyro.getPitchAngleDeg());
	}

	/**
	 * Returns the angle measured by the navX, inverted to adhere to WPILib's axis conventions.
	 */
	public Rotation2d getRoll() {
		return (SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - this.gyro.getRollAngleDeg())
				: Rotation2d.fromDegrees(this.gyro.getRollAngleDeg());
	}

	public double getYawDegPS() {
		return this.gyro.getAngularVelocityDegPS();
	}

	public double getYawRadPS() {
		return this.gyro.getAngularVelocityRadPS();
	}

	/**
	 * Reset the steer motors's internal encoders using the absolute CANCoder measurments.
	 */
	public void resetModulesToAbsolute() {
		for (SwerveModule mod : modules) {
			mod.resetToAbsolute();
		}
	}

	public void crossLockWheels() {
		this.modules[0].setState(
				new SwerveModuleState(0.0, Rotation2d.fromDegrees(SwerveConstants.FrontLeftModule.kCrossAngleDeg)),
				true);
		this.modules[1].setState(
				new SwerveModuleState(0.0, Rotation2d.fromDegrees(SwerveConstants.FrontRightModule.kCrossAngleDeg)),
				true);
		this.modules[2].setState(
				new SwerveModuleState(0.0, Rotation2d.fromDegrees(SwerveConstants.BackLeftModule.kCrossAngleDeg)),
				true);
		this.modules[3].setState(
				new SwerveModuleState(0.0, Rotation2d.fromDegrees(SwerveConstants.BackRightModule.kCrossAngleDeg)),
				true);
	}

	/**
	 * @param desiredPose
	 * @param currentPose
	 * @param tolerance
	 * @return Is the robot within the position tolerance.
	 */
	public boolean withinPositionTolerance(Translation2d desiredPose, Pose2d tolerance) {
		Pose2d currentPose = this.getOdometryPose();
		double xError = desiredPose.getX() - currentPose.getX();
		double yError = desiredPose.getY() - currentPose.getY();
		double angleError = desiredPose.getAngle().getDegrees() - currentPose.getRotation().getDegrees();
		return Math.abs(xError) < tolerance.getX() && Math.abs(yError) < tolerance.getY()
				&& Math.abs(angleError) < tolerance.getRotation().getDegrees();
	}

	/**
	 * @param desiredPose
	 * @param currentPose
	 * @param tolerance
	 * @return Is the robot within the position tolerance.
	 */
	public boolean withinPositionTolerance(Pose2d desiredPose, Pose2d tolerance) {
		Pose2d error = desiredPose.relativeTo(this.getOdometryPose());
		return Math.abs(error.getX()) < tolerance.getX() && Math.abs(error.getY()) < tolerance.getY()
				&& Math.abs(error.getRotation().getDegrees()) < tolerance.getRotation().getDegrees();
	}

	public Command getPathPlannerAutoCommand(String pathGroupName) {
		List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathGroupName,
				SwervePathConstants.kMaxSpeedMPS, SwervePathConstants.kMaxAccelMPSSquared);
		return this.autoBuilder.fullAuto(pathGroup);
	}

	public Command crossLockWheelsCommand() {
		return new RunCommand(this::crossLockWheels, this).until(() -> this.joysticksMoved());
	}

	public boolean joysticksMoved() {
		double translationXValue = RobotContainer.driverA_Controller.getLeftX();
		double translationYValue = RobotContainer.driverA_Controller.getLeftY();
		double rotationValue = RobotContainer.driverA_Controller.getRightX();

		return (translationXValue > RobotContainer.kJoystickDeadband
				|| translationXValue < -RobotContainer.kJoystickDeadband
				|| translationYValue > RobotContainer.kJoystickDeadband
				|| translationYValue < -RobotContainer.kJoystickDeadband
				|| rotationValue > RobotContainer.kJoystickDeadband
				|| rotationValue < -RobotContainer.kJoystickDeadband);
	}

	/**
	 * @param angularVelocityRadPS     - Desired angular velocity as commanded by the drivers.
	 * @param shouldResetAngleSetpoint - Should the current angle be the next setpoint (in teleop this would be set from
	 *                                 the joysticks)
	 * 
	 * @return An adjusted angular velocity accounting for skew.
	 */
	private double calculateAngleCorrectionRadPS(double angularVelocityRadPS) {
		if (!this.isRobotAngleCorrectionEnabled) {
			return angularVelocityRadPS;
		}

		if (angularVelocityRadPS != 0.0) {
			this.runAngleCorrection = false;
			this.angleControlTimer.reset();
			return angularVelocityRadPS;
		}

		if (this.angleControlTimer.hasElapsed(1.0)) {
			if (this.runAngleCorrection) {
				return MathUtil.clamp(
						-this.anglePIDController.calculate(this.gyro.getYawAngleRad(), this.teleopAngleSetpointRad),
						-SwerveConstants.kMaxAngularVelocityRadPS, SwerveConstants.kMaxAngularVelocityRadPS);
			} else {
				this.runAngleCorrection = true;
				this.teleopAngleSetpointRad = this.gyro.getYawAngleRad();
			}
		}

		return angularVelocityRadPS;
	}

	public boolean isChassisMoving() {
		return this.modules[0].getModuleState().speedMetersPerSecond < SwerveConstants.robotIsMovingThresholdMPS
				&& this.modules[1].getModuleState().speedMetersPerSecond < SwerveConstants.robotIsMovingThresholdMPS
				&& this.modules[2].getModuleState().speedMetersPerSecond < SwerveConstants.robotIsMovingThresholdMPS
				&& this.modules[3].getModuleState().speedMetersPerSecond < SwerveConstants.robotIsMovingThresholdMPS;
	}

	/**
	 * The function for putting paths inside the chooser
	 */
	private void createPaths() {
		this.addPath("High Cone & Protector", false, false);
		this.addPath("High Cone & Cube Pickup & Station", false, true);
		this.addPath("High Dropoff & Station", true, true);
		this.addPath("High Cone & Cube", false, true);
		this.addPath("High Cone & Cube & Station", true, false);
		this.addPath("Low Cone & Cube & Station", true, true);
		this.addPath("Low Cone & Cube", false, true);
		this.addPath("Practice Station", true, false);
	}

	/**
	 * @param name          - The name of the path.
	 * @param balanceAtEnd  - Should the robot balance at the end of the path.
	 * @param startWithCube - Should the robot retract instead of home the arm at the start of the path.
	 */
	private void addPath(String name, boolean balanceAtEnd, boolean retractAtStart) {
		ArrayList<Command> commandList = new ArrayList<Command>();
		commandList.add(GrabberSubsystem.getInstance().collectCommand());

		if (retractAtStart) {
			commandList.add(ArmSubsystem.getInstance().retractCommand());
		} else {
			commandList.add(ArmSubsystem.getInstance().autoHomeCommand());
		}

		commandList.add(this.getPathPlannerAutoCommand(name));

		if (balanceAtEnd) {
			commandList.add(new BalanceChassisCommand(this));
		}

		commandList.add(this.crossLockWheelsCommand());

		Command[] commandArray = new Command[commandList.size()];
		commandArray = commandList.toArray(commandArray);
		SwervePathConstants.kPaths.putIfAbsent(name, new SequentialCommandGroup(commandArray));
	}

	@Override
	public void periodic() {
		this.filteredTranslationRatio = this.speedModeRateLimiter.calculate(currentSwerveTranslateRatio);

		this.odometry.update(this.getYaw(), this.getModulesPositions());

		if (Robot.showShuffleboardSubsystemInfo) {
			this.field.setRobotPose(this.getOdometryPose());
		}
	}
}