
package frc.robot.commands.swerve.teleop;

import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDriveCommand extends CommandBase {
	private SwerveSubsystem swerve;
	private DoubleSupplier translationXSupplier, translationYSupplier;
	private DoubleSupplier rotationSupplier;
	private BooleanSupplier isRobotRelativeSupplier;

	public TeleopDriveCommand(SwerveSubsystem swerve, DoubleSupplier translationXSupplier,
			DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
		this(swerve, translationXSupplier, translationYSupplier, rotationSupplier, () -> false);
	}

	public TeleopDriveCommand(SwerveSubsystem swerve, DoubleSupplier translationXSupplier,
			DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier,
			BooleanSupplier isRobotOrientedSupplier) {
		this.swerve = swerve;
		this.addRequirements(this.swerve);

		this.translationXSupplier = translationXSupplier;
		this.translationYSupplier = translationYSupplier;
		this.rotationSupplier = rotationSupplier;
		this.isRobotRelativeSupplier = isRobotOrientedSupplier;
	}

	@Override
	public void initialize() {
		this.swerve.setRobotAngleCorrection(false);
	}

	@Override
	public void execute() {
		// Get joysick values and apply deadband.
		double translationXValue = MathUtil.applyDeadband(translationXSupplier.getAsDouble(),
				RobotContainer.kJoystickDeadband) * this.swerve.getCurrentTranslationSpeedRatio();

		double translationYValue = MathUtil.applyDeadband(translationYSupplier.getAsDouble(),
				RobotContainer.kJoystickDeadband) * this.swerve.getCurrentTranslationSpeedRatio();

		double rotationValue = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), RobotContainer.kJoystickDeadband)
				* this.swerve.getCurrentRotationSpeedRatio();

		boolean isRobotRelative = this.isRobotRelativeSupplier.getAsBoolean();

		// Drive :D
		swerve.teleopDrive(
				new Translation2d(translationXValue, translationYValue).times(SwerveConstants.kChassisMaxSpeedMPS),
				rotationValue * SwerveConstants.kMaxAngularVelocityRadPS, isRobotRelative, true);
	}
}