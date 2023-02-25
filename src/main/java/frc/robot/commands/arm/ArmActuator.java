
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
		this.getTurretToState();
		this.getArmToState();

	}

	// From Zero (Start): Right is clockwise, left is not clockwise
	/**
	 * Schedules the command for the new turret state.
	 */
	private void getTurretToState() {
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

	/**
	 * Schedules the command for the new arm state.
	 */
	private void getArmToState() {
		if (RobotContainer.driverB_Controller.getTouchpadPressed()) {
			this.getArmToState();
		}

		if (RobotContainer.driverB_Controller.getTriangleButton()) {
			this.commandScheduler.schedule(this.arm.setStateCommand(ArmState.kShelf));
		}

		if (RobotContainer.driverB_Controller.getCrossButton()) {
			this.commandScheduler.schedule(this.arm.setStateCommand(ArmState.kInsideRobot));
		}
		if (RobotContainer.driverB_Controller.getSquareButton()) {
			this.commandScheduler.schedule(this.arm.setStateCommand(ArmState.kHigh));
		}
		if (RobotContainer.driverB_Controller.getR1Button()) {
			this.commandScheduler.schedule(this.arm.setStateCommand(ArmState.kLowFront));
		}
		if (RobotContainer.driverB_Controller.getL1Button()) {
			this.commandScheduler.schedule(this.arm.setStateCommand(ArmState.kMid));
		}
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.addDoubleProperty("Desired grid index:", () -> this.gridIndex, null);
	}
}