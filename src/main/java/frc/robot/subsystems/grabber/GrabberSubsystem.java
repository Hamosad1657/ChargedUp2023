
package frc.robot.subsystems.grabber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class GrabberSubsystem extends SubsystemBase {
	private static GrabberSubsystem instance;

	public static GrabberSubsystem getInstance() {
		if (instance == null) {
			instance = new GrabberSubsystem();
		}
		return instance;
	}

	private final CANSparkMax motor;
	private boolean isCollecting;

	private GrabberSubsystem() {
		this.motor = new CANSparkMax(RobotMap.kGrabberMotorID, MotorType.kBrushless);
		motor.setIdleMode(IdleMode.kBrake);
		motor.setSmartCurrentLimit(GrabberConstants.kMaxAmper);

		this.isCollecting = false;

		ShuffleboardTab tab = Shuffleboard.getTab("Arm");
		tab.add("Grabber Motor", this.motor);
	}

	public void onGrabberButtonPressed() {
		if (this.isCollecting) {
			this.motor.set(GrabberConstants.kMotorSpeed);
			this.isCollecting = false;
		} else {
			this.motor.set(-GrabberConstants.kMotorSpeed);
			this.isCollecting = true;
		}
	}

	public void onGrabberButtonReleased() {
		if (!this.isCollecting) {
			this.motor.set(0.0);
		}
	}

	public Command collectCommand() {
		return new RunCommand(() -> this.motor.set(-GrabberConstants.kMotorSpeed), this)
				.withTimeout(GrabberConstants.AutoCollectTime);
	}

	public Command releaseCommand() {
		return new RunCommand(() -> this.motor.set(GrabberConstants.kMotorSpeed), this)
				.withTimeout(GrabberConstants.kAutoReleaseTime).andThen(() -> this.motor.set(0), this);
	}
}
