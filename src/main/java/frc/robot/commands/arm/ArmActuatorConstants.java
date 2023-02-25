
package frc.robot.commands.arm;

import java.util.HashMap;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;

public class ArmActuatorConstants {
	public static final int kKeyboardControllerPort = 2;

	public static final int kUpArrow = 0;
	public static final int kRightArrow = 90;
	public static final int kDownArrow = 180;
	public static final int kLeftArrow = 270;

	public static final double kAngleThreshold = 1;
	public static final double kFrontAngle = 0;

	public static final HashMap<Integer, List<Pose2d>> tagIDtoGrid = new HashMap<Integer, List<Pose2d>>();
	static {
		// TODO: Add each pose for each ID.
		tagIDtoGrid.put(null, null);
	}
}