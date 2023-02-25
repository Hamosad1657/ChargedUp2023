
package com.hamosad1657.lib.sensors;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

/**
 * A wrapper class for kauailabs.navx.frc.AHRS, which adheres to WPILib's coordinate system conventions.
 * <p>
 * 
 * @author Shaked - ask me if you have questions🌠
 */
public class HaNavX implements Sendable {
	private AHRS navX;
	private final Timer commsTimoutTimer = new Timer();
	// KauaiLabs's website says it takes max ~17
	// seconds, so i'm giving it some grace
	private final double kTimeoutSec = 20.0;
	private double yawOffsetDeg = 0.0;

	/*
	 * Waites until the navX is connected and calibrated, or 20 seconds have passed since startup. If the former, print
	 * that the navX is done calibrating and continue; If the latter, print that communication has failed and continue.
	 */
	private void initialize(AHRS navX) {
		this.navX = navX;
		this.navX.enableLogging(true);
		this.commsTimoutTimer.start();

		// Wait until navX is connected or 20 seconds have passed
		while (!this.navX.isConnected()) {
			if (this.commsTimoutTimer.hasElapsed(kTimeoutSec)) {
				break;
			}
		}
		// Wait until navX is calibrated or 20 seconds have passed
		while (this.navX.isCalibrating()) {
			if (this.commsTimoutTimer.hasElapsed(kTimeoutSec)) {
				break;
			}
		}

		if (this.commsTimoutTimer.hasElapsed(kTimeoutSec)) {
			Robot.print("Failed to connect to navX within 20 seconds from startup.");
			DriverStation.reportError("Failed to connect to navX within 20 seconds from startup.", false);
		} else {
			Robot.print("navX done calibrating.");
		}

		this.commsTimoutTimer.stop();
	}

	/**
	 * Starts communtication between navX and RoboRIO, enables logging to the RioLog & Driver Station, waits until the
	 * navX is connected and calibrated, then returns an instance.
	 *
	 * @param port serial port (usually USB)
	 * @return an instance of HaNavx, which wrapps AHRS.
	 */
	public HaNavX(SerialPort.Port port) {
		try {
			this.initialize(new AHRS(port, SerialDataType.kProcessedData, (byte) 60));
		} catch (RuntimeException E) {
			Robot.print("Failed to connect to navX.");
		}
	}

	/**
	 * Starts communtication between navX and RoboRIO, enables logging to the RioLog & Driver Station, waits until the
	 * navX is connected and calibrated, then returns an instance.
	 * 
	 * @param port I2C port. Using the onboard I2C port is not reccomended, for more information click here:
	 *             https://docs.wpilib.org/en/stable/docs/yearly-overview/known-issues.html#onboard-i2c-causing-system-lockups
	 * @return an instance of HaNavx, which wrapps AHRS.
	 */
	public HaNavX(I2C.Port port) {
		try {
			this.initialize(new AHRS(port));
		} catch (RuntimeException E) {
			Robot.print("Failed to connect to navX.");
		}
	}

	/**
	 * Starts communtication between navX and RoboRIO, enables logging to the RioLog & Driver Station, waits until the
	 * navX is connected and calibrated, then returns an instance.
	 * 
	 * @param port SPI port
	 * @return an instance of HaNavx, which wrapps AHRS.
	 */
	public HaNavX(SPI.Port port) {
		try {
			this.initialize(new AHRS(port));
		} catch (RuntimeException E) {
			Robot.print("Failed to connect to navX.");
		}
	}

	/**
	 * Used to set the angle the navX is currently facing as zero.
	 */
	public void zeroYaw() {
		this.yawOffsetDeg = 0.0;
		try {
			this.navX.zeroYaw();
		} catch (RuntimeException E) {
			Robot.print("Failed to zero navX yaw.");
		}
	}

	/**
	 * Used to set the angle the navX is currently facing minus the offset as zero.
	 */
	public void zeroYaw(double offsetDeg) {
		this.yawOffsetDeg = offsetDeg;
	}

	/**
	 * Used to set the angle the navX is currently facing minus the offset as zero.
	 */
	public void zeroYaw(Rotation2d offsetDeg) {
		this.yawOffsetDeg = offsetDeg.getDegrees();
	}

	/**
	 * @return The angle of the navX on the Z axis (perpendicular to earth, "yaw") in degrees. Rotating
	 *         counter-clockwise makes the the angle increase, and rotating clockwise makes the angle decrease,
	 *         according to WPILib's coordinate system conventions.
	 */
	public double getYawAngleDeg() {
		try {
			return -this.navX.getYaw() - this.yawOffsetDeg;
		} catch (RuntimeException e) {
			return 0.0;
		}
	}

	/**
	 * @return The angle of the navX on the Z axis (perpendicular to earth, "yaw") in radians. Rotating
	 *         counter-clockwise makes the angle increase, and rotating clockwise makes the angle decrease, according to
	 *         WPILib's coordinate system conventions.
	 */
	public double getYawAngleRad() {
		return Math.toRadians(this.getYawAngleDeg());
	}

	/**
	 * @return The angle of the navX on the Z axis (perpendicular to earth, "yaw") as a Rotation2d. Rotating
	 *         counter-clockwise makes the angle increase, and rotating clockwise makes the angle decrease, according to
	 *         WPILib's coordinate system conventions.
	 */
	public Rotation2d getYawRotation2d() {
		return Rotation2d.fromDegrees(this.getYawAngleDeg());
	}

	/**
	 * 
	 * @return The angle of the navX on the X axis (left-right tilt) in degrees. Tilting left makes the angle increase,
	 *         and tilting right makes the angle decrease. If the angle returned is incorrect, verify that the navX
	 *         axises are matching to the robot axises, or use the omnimount feature (as specified in kauailabs's
	 *         website).
	 *
	 */
	public double getPitchAngleDeg() {
		try {
			return this.navX.getPitch();
		} catch (RuntimeException E) {
			return 0.0;
		}
	}

	/**
	 * @return The angle of the navX on the X axis (left-right tilt) in radians. Tilting left makes the angle increase,
	 *         and tilting right makes the angle decrease. If the angle returned is incorrect, verify that the navX
	 *         axises are matching to the robot axises, or use the omnimount feature (as specified in kauailabs's
	 *         website).
	 */
	public double getPitchAngleRad() {
		return Math.toRadians(this.getPitchAngleDeg());
	}

	/**
	 * @return The angle of the navX on the X axis (left-right tilt) as a Rotation2d. Tilting left makes the angle
	 *         increase, and tilting right makes the angle decrease. If the angle returned is incorrect, verify that the
	 *         navX axises are matching to the robot axises, or use the omnimount feature (as specified in kauailabs's
	 *         website).
	 */
	public Rotation2d getPitchRotation2d() {
		return Rotation2d.fromDegrees(this.getPitchAngleDeg());
	}

	/**
	 * @return The angle of the navX on the Y axis (forward-backward tilt) in degrees. Tilting forwards makes the angle
	 *         increase, and tilting backwards makes the angle decrease. If the angle returned is incorrect, verify that
	 *         the navX axises are matching to the robot axises, or use the omnimount feature (as specified in
	 *         kauailabs's website).
	 */
	public double getRollAngleDeg() {
		try {
			return this.navX.getRoll();
		} catch (RuntimeException E) {
			return 0.0;
		}
	}

	/**
	 * @return The angle of the navX on the Y axis (forward-backward tilt) in radians. Tilting forwards makes the angle
	 *         increase, and tilting backwards makes the angle decrease. If the angle returned is incorrect, verify that
	 *         the navX axises are matching to the robot axises, or use the omnimount feature (as specified in
	 *         kauailabs's website).
	 */
	public double getRollAngleRad() {
		return Math.toRadians(this.getRollAngleDeg());
	}

	/**
	 * @return The angle of the navX on the Y axis (forward-backward tilt) as a Rotation2d. Tilting forwards makes the
	 *         angle increase, and tilting backwards makes the angle decrease. If the angle returned is incorrect,
	 *         verify that the navX axises are matching to the robot axises, or use the omnimount feature (as specified
	 *         in kauailabs's website).
	 */
	public Rotation2d getRollRotation2d() {
		return Rotation2d.fromDegrees(this.getRollAngleDeg());
	}

	/**
	 * @return the rate of change in the angle on the Z axis (perpendicular to earth, "yaw") in degrees. Rotating
	 *         counter-clockwise returns a positive value, and clockwise returns a negative value.
	 */
	public double getAngularVelocityDegPS() {
		try {
			return -this.navX.getRate();
		} catch (RuntimeException E) {
			return 0.0;
		}
	}

	/**
	 * @return the rate of change in the angle on the Z axis (perpendicular to earth, "yaw") in radians. Rotating
	 *         counter-clockwise returns a positive value, and clockwise returns a negative value.
	 */
	public double getAngularVelocityRadPS() {
		return Math.toRadians(-this.navX.getRate());
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("HaNavx");
		builder.addDoubleProperty("YawAngleDeg", this::getYawAngleDeg, null);
		builder.addDoubleProperty("YawAngleRad", this::getYawAngleRad, null);

		// builder.addDoubleProperty("AngularVelocityDegPS",
		// this::getAngularVelocityDegPS, null);
		// builder.addDoubleProperty("AngularVelocityRadPS",
		// this::getAngularVelocityRadPS, null);
	}
}