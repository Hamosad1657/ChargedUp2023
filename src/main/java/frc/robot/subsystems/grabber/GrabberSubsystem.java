
package frc.robot.subsystems.grabber;

import com.hamosad1657.lib.motors.HaCANSparkMax;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

	private final HaCANSparkMax grabberMotor;

	private GrabberSubsystem() {
		CANSparkMax motor = new CANSparkMax(RobotMap.kGrabberMotorID, MotorType.kBrushless);
		motor.setSmartCurrentLimit(GrabberConstants.kMaxAmper);
		this.grabberMotor = new HaCANSparkMax(motor);

		ShuffleboardTab tab = Shuffleboard.getTab("Arm");
		tab.add("Grabber Motor", this.grabberMotor);
	}


}
