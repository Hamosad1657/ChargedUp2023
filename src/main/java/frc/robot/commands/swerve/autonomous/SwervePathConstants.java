
package frc.robot.commands.swerve.autonomous;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public final class SwervePathConstants {
	public static final SwerveSubsystem swerve = SwerveSubsystem.getInstance();

	// For use with getPathFollowingCommand
	public static final HashMap<String, Command> kPathCommandsMap = new HashMap<String, Command>();
	static {
		kPathCommandsMap.put("Print Event 1", new InstantCommand(() -> Robot.print("Print 1 ")));
		kPathCommandsMap.put("Print Event 2", new InstantCommand(() -> Robot.print("Print 2 ")));
		kPathCommandsMap.put("Cross Lock Wheels", new InstantCommand(SwervePathConstants.swerve::crossLockWheels));
	}
}
