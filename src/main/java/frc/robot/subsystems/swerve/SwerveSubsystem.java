
package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fusionLib.swerve.SwerveModule;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.commands.swerve.autonomous.SwervePathConstants;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.function.Consumer;
import java.util.stream.Stream;
import com.hamosad1657.lib.sensors.HaNavX;
import com.hamosad1657.lib.vision.limelight.Limelight;
import com.hamosad1657.lib.vision.limelight.LimelightConstants;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

public class SwerveSubsystem extends SubsystemBase {
	private static SwerveSubsystem instance;

	/** Get the singleton instance of this subsystem. */
	public static SwerveSubsystem getInstance() {
		if (instance == null) {
			instance = new SwerveSubsystem();
		}
		return instance;
	}

	private final double robotIsMovingThresholdMPS = 0.1;
	private final Timer angleControlTimer;
	/** A PIDController for fixing the robot angle skew. */
	private final Field2d field;
	private final PIDController anglePIDController;
	private final ShuffleboardTab swerveTab, odometryTab;
	private final ShuffleboardLayout frontLeftModuleList, frontRightModuleList, backLeftModuleList, backRightModuleList,
			odometryList;
	private final GenericEntry frontLeftAngle, frontLeftSetpoint, frontRightAngle, frontRightSetpoint, backLeftAngle,
			backLeftSetpoint, backRightAngle, backRightSetpoint, frontLeftSpeed, frontRightSpeed, backLeftSpeed,
			backRightSpeed, frontLeftError, frontRightError, backLeftError, backRightError, anglePIDRunningEntry,
			estimatorXEntry, estimatorYEntry, odometryXEntry, odometryYEntry, limelightXEntry, limelightYEntry,
			teleopAngleErrorEntry;
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
	private final SwerveDriveOdometry odometry;
	private final SwerveDrivePoseEstimator poseEstimator;
	private final HaNavX gyro;

	private double teleopAngleSetpointRad;
	private boolean isRobotAngleCorrectionEnabled = false, runAngleCorrection = false;
	public double currentSwerveTranslateRatio = SwerveConstants.kSwerveTranslateRatioFast;
	public double currentSwerveRotationRatio = SwerveConstants.kSwerveSpinRatioFast;

	private final SwerveAutoBuilder autoBuilder;

	private SwerveSubsystem() {
		this.gyro = new HaNavX(RobotMap.kNavXPort);
		this.zeroGyro();

		this.modules = new SwerveModule[] { new SwerveModule(0, SwerveConstants.FrontLeftModule.constants),
				new SwerveModule(1, SwerveConstants.FrontRightModule.constants),
				new SwerveModule(2, SwerveConstants.BackLeftModule.constants),
				new SwerveModule(3, SwerveConstants.BackRightModule.constants) };
		this.angleControlTimer = new Timer();
		this.angleControlTimer.start();

		this.anglePIDController = SwerveConstants.kRobotAnglePIDGains.toPIDController();
		this.anglePIDController.enableContinuousInput(-Math.PI, Math.PI);
		this.anglePIDController.setTolerance(SwerveConstants.kRobotAngleToleranceRad);

		this.swerveTab = Shuffleboard.getTab("Swerve");
		this.odometryTab = Shuffleboard.getTab("Odometry");

		this.anglePIDRunningEntry = this.swerveTab.add("Angle PID Running", false).getEntry();
		this.teleopAngleErrorEntry = this.swerveTab.add("Angle PID Error", 0.0).getEntry();

		this.swerveTab.add("Gyro", this.gyro);
		this.swerveTab.add("Angle PID", this.anglePIDController);
		this.frontLeftModuleList = this.swerveTab.getLayout("Front Left Module", BuiltInLayouts.kList)
				.withPosition(0, 0).withSize(2, 3);
		this.frontLeftAngle = this.frontLeftModuleList.add("Angle", 0.0).getEntry();
		this.frontLeftSetpoint = this.frontLeftModuleList.add("Setpoint", 0.0).getEntry();
		this.frontLeftSpeed = this.frontLeftModuleList.add("Speed", 0.0).getEntry();
		this.frontLeftError = this.frontLeftModuleList.add("error", 0.0).getEntry();

		this.frontRightModuleList = this.swerveTab.getLayout("Front Right Module", BuiltInLayouts.kList)
				.withPosition(2, 0).withSize(2, 3);
		this.frontRightAngle = this.frontRightModuleList.add("Angle", 0.0).getEntry();
		this.frontRightSetpoint = this.frontRightModuleList.add("Setpoint", 0.0).getEntry();
		this.frontRightSpeed = this.frontRightModuleList.add("Speed", 0.0).getEntry();
		this.frontRightError = this.frontRightModuleList.add("error", 0.0).getEntry();

		this.backLeftModuleList = this.swerveTab.getLayout("Back Left Module", BuiltInLayouts.kList).withPosition(4, 0)
				.withSize(2, 3);
		this.backLeftAngle = this.backLeftModuleList.add("Angle", 0.0).getEntry();
		this.backLeftSetpoint = this.backLeftModuleList.add("Setpoint", 0.0).getEntry();
		this.backLeftSpeed = this.backLeftModuleList.add("Speed", 0.0).getEntry();
		this.backLeftError = this.backLeftModuleList.add("error", 0.0).getEntry();

		this.backRightModuleList = this.swerveTab.getLayout("Back Right Module", BuiltInLayouts.kList)
				.withPosition(6, 0).withSize(2, 3);
		this.backRightAngle = this.backRightModuleList.add("Angle", 0.0).getEntry();
		this.backRightSetpoint = this.backRightModuleList.add("Setpoint", 0.0).getEntry();
		this.backRightSpeed = this.backRightModuleList.add("Speed", 0.0).getEntry();
		this.backRightError = this.backRightModuleList.add("error", 0.0).getEntry();

		this.field = new Field2d();
		this.odometryTab.add("Field", this.field);

		this.odometryList = this.odometryTab.getLayout("Odometry", BuiltInLayouts.kList).withPosition(0, 0).withSize(2,
				4);
		this.estimatorXEntry = this.odometryList.add("Estimator X", 0.0).getEntry();
		this.estimatorYEntry = this.odometryList.add("Estimator Y", -1657).withWidget(BuiltInWidgets.kTextView)
				.getEntry();
		this.odometryXEntry = this.odometryList.add("Odometry X", -1657).withWidget(BuiltInWidgets.kTextView)
				.getEntry();
		this.odometryYEntry = this.odometryList.add("Odometry Y", -1657).withWidget(BuiltInWidgets.kTextView)
				.getEntry();
		this.limelightXEntry = this.odometryList.add("Limelight X", -1657).withWidget(BuiltInWidgets.kTextView)
				.getEntry();
		this.limelightYEntry = this.odometryList.add("Limelight Y", -1657).withWidget(BuiltInWidgets.kTextView)
				.getEntry();
		/*
		 * By pausing init for a second before setting module offsets, we avoid a bug with inverting motors. See
		 * https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
		 */
		Timer.delay(1.0);
		resetModulesToAbsolute();

		this.odometry = new SwerveDriveOdometry(SwerveConstants.kSwerveKinematics, this.getYaw(),
				this.getModulesPositions(), SwervePathConstants.kStartPose);

		this.poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.kSwerveKinematics, this.getYaw(),
				this.getModulesPositions(), SwervePathConstants.kStartPose);

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

		SwervePathConstants.createCommands();
		this.createPaths();
	}

	/**
	 * Drive the swerve.
	 * 
	 * @param translation   - A field-relative or robot-relative direction+speed to move in.
	 * @param rotationRadPS - A rotation speed in RadPS. Open-loop
	 * @param robotRelative - Whether the passed Translation2d is field-relative or robot-relative (true if robot
	 *                      relative).
	 * @param isOpenLoop    - Is the wheel speed controlled open-loop or closed-loop.
	 */
	public void teleopDrive(Translation2d translation, double rotationRadPS, boolean robotRelative,
			boolean isOpenLoop) {
		// Set the swerveModuleStates from the desired ChassisSpeeds using kinematics.
		SwerveModuleState[] swerveModuleStates = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
				// If field relative, convert to robot-relative ChassisSpeeds with gyro.
				// If robot relative, just convert to ChassisSpeeds.
				robotRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(),
								this.calculateAngleCorrectionRadPS(rotationRadPS), this.getYaw())
						: new ChassisSpeeds(translation.getX(), translation.getY(),
								this.calculateAngleCorrectionRadPS(rotationRadPS)));

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
			this.currentSwerveRotationRatio = SwerveConstants.kSwerveSpinRatioSlow;
		} else {
			this.currentSwerveTranslateRatio = SwerveConstants.kSwerveTranslateRatioFast;
			this.currentSwerveRotationRatio = SwerveConstants.kSwerveSpinRatioFast;
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
	 * @return The robot's position on the field as measured by the pose estimator, which uses both vision and odometry.
	 *         Units in meters and Rotation2d.
	 */
	public Pose2d getEstimatedPose() {
		return this.poseEstimator.getEstimatedPosition();
	}

	/**
	 * Resets the estimated robot position on the field.
	 * <p>
	 * ONLY CALL IF:
	 * <p>
	 * It's the first auto path and you're passing the path start pose, or if you know where the robot is with 100%
	 * confidence.
	 * 
	 * @param newPose - Position on the field. Units in meters and Rotation2d.
	 */
	public void resetEstimatedPose(Pose2d newPose) {
		this.poseEstimator.resetPosition(this.getYaw(), this.getModulesPositions(), newPose);
	}

	/**
	 * @return The robot position as measured by the odometry. Units in meters and Rotation2d.
	 */
	public Pose2d getOdometryPose() {
		return this.odometry.getPoseMeters();
	}

	/** Set the odometry to a specific position. Units in meters and Rotation2d. */
	public void resetOdometry(Pose2d pose) {
		this.odometry.resetPosition(this.getYaw(), this.getModulesPositions(), pose);
		Robot.print("Odometry reset to: " + Double.toString(this.getOdometryPose().getX())
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
		this.gyro.zeroYaw();
		this.teleopAngleSetpointRad = 0.0;
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
				: Rotation2d.fromDegrees(this.gyro.getYawAngleDeg());
	}

	/**
	 * Returns the angle measured by the navX, inverted to adhere to WPILib's axis conventions.
	 */
	public Rotation2d getRoll() {
		return (SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - this.gyro.getRollAngleDeg())
				: Rotation2d.fromDegrees(this.gyro.getYawAngleDeg());
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
		Pose2d currentPose = this.getEstimatedPose();
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
		Pose2d error = desiredPose.relativeTo(this.getEstimatedPose());
		return Math.abs(error.getX()) < tolerance.getX() && Math.abs(error.getY()) < tolerance.getY()
				&& Math.abs(error.getRotation().getDegrees()) < tolerance.getRotation().getDegrees();
	}

	public CommandBase getPathPlannerAutoCommand(String pathGroupName) {
		List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathGroupName,
				SwervePathConstants.kMaxSpeedMPS, SwervePathConstants.kMaxAccelMPSSquared);
		return this.autoBuilder.fullAuto(pathGroup);
	}

	public Command crossLockWheelsCommand() {
		return new RunCommand(this::crossLockWheels, this).until(RobotContainer::shouldRobotMove);
	}

	/**
	 * @param angularVelocityRadPS     - Desired angular velocity as commanded by the drivers.
	 * @param shouldResetAngleSetpoint - Should the current angle be the next setpoint (in teleop this would be set from
	 *                                 the joysticks)
	 * 
	 * @return An adjusted angular velocity accounting for skew.
	 */
	private double calculateAngleCorrectionRadPS(double angularVelocityRadPS) {
		this.teleopAngleErrorEntry.setDouble(Math.toDegrees(this.anglePIDController.getPositionError()));

		if (!this.isRobotAngleCorrectionEnabled) {
			this.anglePIDRunningEntry.setBoolean(false);
			return angularVelocityRadPS;
		}

		if (angularVelocityRadPS != 0.0) {
			this.runAngleCorrection = false;
			this.angleControlTimer.reset();
			this.anglePIDRunningEntry.setBoolean(false);
			return angularVelocityRadPS;
		}

		if (this.angleControlTimer.hasElapsed(1.0)) {
			if (this.runAngleCorrection) {
				this.anglePIDRunningEntry.setBoolean(true);
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
		return this.modules[0].getModuleState().speedMetersPerSecond < this.robotIsMovingThresholdMPS
				&& this.modules[1].getModuleState().speedMetersPerSecond < this.robotIsMovingThresholdMPS
				&& this.modules[2].getModuleState().speedMetersPerSecond < this.robotIsMovingThresholdMPS
				&& this.modules[3].getModuleState().speedMetersPerSecond < this.robotIsMovingThresholdMPS;
	}

	/**
	 * The function for putting paths inside the chooser
	 */
	private void createPaths() {
		try (Stream<Path> paths = Files.walk(Paths.get("/home/you/Desktop"))) {
			paths.forEach(new Consumer<Path>() {
				@Override
				public void accept(Path path) {
					String name = path.toString().replace(".path", "");
					SwervePathConstants.kPaths.put(name,
							new SequentialCommandGroup(getPathPlannerAutoCommand(name), crossLockWheelsCommand()));
				}
			});
		} catch (Exception e) {
			Robot.print(e);
		}
	}

	@Override
	public void periodic() {
		this.odometry.update(this.getYaw(), this.getModulesPositions());
		this.field.setRobotPose(this.getEstimatedPose());

		if (Limelight.hasTargets(LimelightConstants.kLimelightName)) {
			// Set robot pose depending on alliance
			Pose3d robotPose3d = (DriverStation.getAlliance() == Alliance.Blue)
					? Limelight.getBotpose_wpiBlue(LimelightConstants.kLimelightName)
					: Limelight.getBotpose_wpiBlue(LimelightConstants.kLimelightName);
			if (robotPose3d != null) {
				Pose2d robotPose = new Pose2d(robotPose3d.getX(), robotPose3d.getY(), this.getYaw());
				this.poseEstimator.addVisionMeasurement(robotPose, Timer.getFPGATimestamp());

				this.limelightXEntry.setDouble(robotPose.getX());
				this.limelightYEntry.setDouble(robotPose.getY());
			}
		}
		this.poseEstimator.updateWithTime(Timer.getFPGATimestamp(), this.getYaw(), this.getModulesPositions());

		for (SwerveModule mod : modules) {
			SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
			SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated",
					mod.getModulePosition().angle.getDegrees());
			SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity",
					mod.getModuleState().speedMetersPerSecond);
		}

		this.frontLeftAngle.setDouble(this.modules[0].getAngle().getDegrees());
		this.frontLeftSetpoint.setDouble(this.modules[0].getDesiredAngle().getDegrees());
		this.frontLeftSpeed.setDouble(this.modules[0].getModuleState().speedMetersPerSecond);
		this.frontLeftError
				.setDouble(this.modules[0].getDesiredAngle().getDegrees() - this.modules[0].getAngle().getDegrees());

		this.frontRightAngle.setDouble(this.modules[1].getAngle().getDegrees());
		this.frontRightSetpoint.setDouble(this.modules[1].getDesiredAngle().getDegrees());
		this.frontRightSpeed.setDouble(this.modules[1].getModuleState().speedMetersPerSecond);
		this.frontRightError
				.setDouble(this.modules[1].getDesiredAngle().getDegrees() - this.modules[1].getAngle().getDegrees());

		this.backLeftAngle.setDouble(this.modules[2].getAngle().getDegrees());
		this.backLeftSetpoint.setDouble(this.modules[2].getDesiredAngle().getDegrees());
		this.backLeftSpeed.setDouble(this.modules[2].getModuleState().speedMetersPerSecond);
		this.backLeftError
				.setDouble(this.modules[2].getDesiredAngle().getDegrees() - this.modules[2].getAngle().getDegrees());

		this.backRightAngle.setDouble(this.modules[3].getAngle().getDegrees());
		this.backRightSetpoint.setDouble(this.modules[3].getDesiredAngle().getDegrees());
		this.backRightSpeed.setDouble(this.modules[3].getModuleState().speedMetersPerSecond);
		this.backRightError
				.setDouble(this.modules[3].getDesiredAngle().getDegrees() - this.modules[3].getAngle().getDegrees());
		this.estimatorXEntry.setDouble(this.getEstimatedPose().getX());
		this.estimatorYEntry.setDouble(this.getEstimatedPose().getY());
		this.odometryXEntry.setDouble(this.getOdometryPose().getX());
		this.odometryYEntry.setDouble(this.getOdometryPose().getY());
	}

}