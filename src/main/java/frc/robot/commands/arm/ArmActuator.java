
package frc.robot.commands.arm;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmConstants.ArmState;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.subsystems.turret.TurretSubsystem;

public class ArmActuator implements Sendable {
	private static ArmActuator instance;

	public static ArmActuator getInstance() {
		if (instance == null) {
			instance = new ArmActuator();
		}
		return instance;
	}

	private final PS4Controller controller;
	private final CommandScheduler commandScheduler;
	private final TurretSubsystem turret;
	private final ArmSubsystem arm;

	private ArmState desiredArmState;
	private int gridIndex;
	private boolean turretRotationSide; // true = right, false = left

	private ArmActuator() {
		this.controller = new PS4Controller(ArmActuatorConstants.kKeyboardControllerPort);

		this.commandScheduler = CommandScheduler.getInstance();
		this.turret = TurretSubsystem.getInstance();
		this.arm = ArmSubsystem.getInstance();

		this.gridIndex = 0;
	}

	/**
	 * Gets the grid index from the controller and moves the arm and chassis to the required position.
	 * 
	 * Needs to run periodically.
	 */
	public void run() {
		this.getGridIndexFromController();
		this.getTurretToState();

		if (RobotContainer.driverB_Controller.getTouchpadPressed()) {
			this.getArmToState();
		}

		if (RobotContainer.driverB_Controller.getTriangleButton()) {
			this.commandScheduler.schedule(this.arm.setStateCommand(ArmState.kShelf));
		}

		if (RobotContainer.driverB_Controller.getCrossButton()) {
			this.commandScheduler.schedule(this.arm.setStateCommand(ArmState.kInsideRobot));
		}
	}

	/**
	 * Gets the grid index from the controller.
	 */
	private void getGridIndexFromController() {
		for (int i = 1; i <= 9; i++) {
			if (controller.getRawButton(i)) {
				this.gridIndex = i;
			}
		}
	}

	/**
	 * Schedules the command for the new arm state.
	 */
	public void getArmToState() {
		if (this.gridIndex == 7 || this.gridIndex == 8 || this.gridIndex == 9) {
			this.desiredArmState = ArmConstants.ArmState.kHigh;
		} else if (this.gridIndex == 4 || this.gridIndex == 5 || this.gridIndex == 6) {
			this.desiredArmState = ArmConstants.ArmState.kMid;
		} else if (this.gridIndex == 1 || this.gridIndex == 2 || this.gridIndex == 3) {
			this.desiredArmState = ArmConstants.ArmState.kLowFront;
		} else {
			this.desiredArmState = ArmConstants.ArmState.kInsideRobot;
		}

		this.commandScheduler.schedule(this.arm.setStateCommand(this.desiredArmState));
	}

	// From Zero (Start): Right is clockwise, left is not clockwise
	/**
	 * Schedules the command for the new turret state.
	 */
	public void getTurretToState() {
		switch (RobotContainer.driverB_Controller.getPOV()) {
		case ArmActuatorConstants.kLeftArrow:
			this.turretRotationSide = false;
			break;
		case ArmActuatorConstants.kRightArrow:
			this.turretRotationSide = true;
			break;
		case ArmActuatorConstants.kUpArrow:
			if (this.turret.getAngle() > ArmActuatorConstants.kFrontAngle - ArmActuatorConstants.kAngleThreshold
					|| this.turret.getAngle() < ArmActuatorConstants.kFrontAngle
							+ ArmActuatorConstants.kAngleThreshold) {
				this.commandScheduler.schedule(this.turret.setTurretAngle(ArmActuatorConstants.kFrontAngle));

			}
			break;
		case ArmActuatorConstants.kDownArrow:
			if (this.turretRotationSide) { // Right
				if (this.turret.getAngle() < TurretConstants.kCANCoderCWLimitDeg
						+ ArmActuatorConstants.kAngleThreshold) {
					this.commandScheduler.schedule(this.turret.setTurretAngle(
							TurretConstants.kCANCoderCWLimitDeg + ArmActuatorConstants.kAngleThreshold));
				}
			} else {// Left
				if (this.turret.getAngle() > TurretConstants.kCANCoderCCWLimitDeg
						- ArmActuatorConstants.kAngleThreshold) {
					this.commandScheduler.schedule(this.turret.setTurretAngle(
							TurretConstants.kCANCoderCWLimitDeg - ArmActuatorConstants.kAngleThreshold));
				}
			}
			break;
		}
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Desired grid index:", () -> this.gridIndex, null);
	}
}