
package com.hamosad1657.lib.motors;

import com.hamosad1657.lib.math.HaUnitConvertor;
import com.hamosad1657.lib.math.HaUnits.PIDGains;
import com.hamosad1657.lib.math.HaUnits.Position;
import com.hamosad1657.lib.math.HaUnits.Velocity;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.util.sendable.SendableBuilder;

public class HaCANSparkMax extends HaMotorController {
	// Note: The spark max takes velocity setpoint in RPM, and position setpoint in
	// rotations.

	private static final double kWheelRadNone = -1;
	private static final double kMaxPossibleMotorCurrent = 40.0;

	private final double currentLimit;
	public CANSparkMax motor;

	private SparkMaxPIDController PIDController;
	private double wheelRadiusMeters;
	private RelativeEncoder encoder;

	private String controlType;
	private double controlRefrence;

	/**
	 * @param motor        - A CANSparkMax object.
	 * @param pidGains
	 * @param wheelRadiusM
	 * @param maxAmpere    - 40 by default.
	 */
	public HaCANSparkMax(CANSparkMax motor, PIDGains pidGains, double wheelRadiusM, double maxAmpere) {
		this.motor = motor;
		this.PIDController = this.motor.getPIDController();
		this.configPID(pidGains);
		this.wheelRadiusMeters = wheelRadiusM;
		this.controlType = "None";
		this.controlRefrence = 0;
		this.encoder = this.motor.getEncoder();
		this.currentLimit = maxAmpere;
	}

	public HaCANSparkMax(CANSparkMax motor, PIDGains PIDGains) {
		this(motor, PIDGains, kWheelRadNone, kMaxPossibleMotorCurrent);
	}

	public HaCANSparkMax(CANSparkMax motor, PIDGains PIDGains, double maxAmpere) {
		this(motor, PIDGains, kWheelRadNone, maxAmpere);
	}

	public HaCANSparkMax(CANSparkMax motor, double wheelRadiusM) {
		this(motor, new PIDGains(), wheelRadiusM, kMaxPossibleMotorCurrent);
	}

	public HaCANSparkMax(CANSparkMax motor, double wheelRadiusM, double maxAmpere) {
		this(motor, new PIDGains(), wheelRadiusM, maxAmpere);
	}

	public HaCANSparkMax(CANSparkMax motor) {
		this(motor, new PIDGains(), kWheelRadNone, kMaxPossibleMotorCurrent);
	}

	public HaCANSparkMax(int motorID) {
		this(new CANSparkMax(motorID, MotorType.kBrushless), new PIDGains(), kWheelRadNone, kMaxPossibleMotorCurrent);
	}

	@Override
	public void configPID(PIDGains pidGains) {
		this.PIDController.setP(pidGains.p);
		this.PIDController.setI(pidGains.i);
		this.PIDController.setD(pidGains.d);
		this.PIDController.setFF(pidGains.ff);
		this.PIDController.setIZone(pidGains.iZone);
	}

	@Override
	public void set(double value, Velocity type) {
		this.controlRefrence = value;
		switch (type) {
		case kMPS:
			value = HaUnitConvertor.MPSToRPM(value, this.wheelRadiusMeters);
			this.controlType = "Meters per second (velocity)";
			break;
		case kRPM:
			this.controlType = "Rotation per minute (velocity)";
			break;
		case kDegPS:
			value = HaUnitConvertor.degPSToRPM(value);
			this.controlType = "Degrees per second (velocity)";
			break;
		case kRadPS:
			value = HaUnitConvertor.radPSToRPM(value);
			this.controlType = "Radians per second (velocity)";
			break;
		}

		this.PIDController.setReference(value, ControlType.kVelocity);
	}

	@Override
	public double get(Velocity type) {
		switch (type) {
		case kMPS:
			return HaUnitConvertor.RPMToMPS(this.encoder.getVelocity(), this.wheelRadiusMeters);
		case kRPM:
			return this.encoder.getVelocity();
		case kDegPS:
			return HaUnitConvertor.RPMToDegPS(this.encoder.getVelocity());
		case kRadPS:
			return HaUnitConvertor.RPMToRadPS(this.encoder.getVelocity());
		default:
			return this.encoder.getVelocity();
		}
	}

	@Override
	public void set(double value, Position type) {
		this.controlRefrence = value;
		switch (type) {
		case kDeg:
			value = value / 360;
			this.controlType = "Degrees (position)";
			break;
		case kRad:
			value = value / (Math.PI * 2);
			this.controlType = "Radians (position)";
			break;
		case kRot:
			this.controlType = "Rotations (position)";
			break;
		}
		this.PIDController.setReference(value, ControlType.kPosition);
	}

	@Override
	public double get(Position type) {
		switch (type) {
		case kDeg:
			return this.encoder.getPosition() * 360;
		case kRad:
			return this.encoder.getPosition() * (Math.PI * 2);
		case kRot:
			return this.encoder.getPosition();
		}
		return 0;
	}

	@Override
	public void set(double value) {
		if (value >= -1.0 && value <= 1.0) {
			this.controlRefrence = value;
			this.controlType = "Percent output";
			this.motor.set(value);
		}
	}

	@Override
	public double get() {
		return this.motor.get();
	}

	@Override
	public void setCurrent(double value) {
		if (value >= 0 && value <= this.currentLimit) {
			this.PIDController.setReference(value, ControlType.kCurrent);
		}
	}

	@Override
	public double getCurrent() {
		return this.motor.getOutputCurrent();
	}

	@Override
	public void setIdleMode(IdleMode idleMode) {
		this.motor.setIdleMode(idleMode);
	}

	@Override
	protected String getIdleMode() {
		return this.motor.getIdleMode().name();
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("RobotPreferences");

		builder.addStringProperty("Control Type", this::getControlType, null);
		builder.addStringProperty("Control Reference", null, null);

		builder.addDoubleProperty("Motor ID", this.motor::getDeviceId, null);
		builder.addDoubleProperty("Speed [-1.0, 1.0]", this::get, this::set);
		builder.addDoubleProperty("Current", this::getCurrent, this::setCurrent);
		builder.addStringProperty("IdleMode", this::getIdleMode, null);

		builder.addDoubleProperty("P", this.PIDController::getP, this.PIDController::setP);
		builder.addDoubleProperty("I", this.PIDController::getI, this.PIDController::setI);
		builder.addDoubleProperty("D", this.PIDController::getD, this.PIDController::setD);
		builder.addDoubleProperty("FF", this.PIDController::getFF, this.PIDController::setFF);
		builder.addDoubleProperty("IZone", this.PIDController::getIZone, this.PIDController::setIZone);
	}

	@Override
	public void setEncoderPosition(double value, Position type) {
		switch (type) {
		case kDeg:
			this.encoder.setPosition(value / 360);
			break;
		case kRad:
			this.encoder.setPosition(Math.PI * 2);
			break;
		case kRot:
			this.encoder.setPosition(value);
			break;
		}
	}

	@Override
	protected String getControlType() {
		return this.controlType;
	}

	@Override
	protected double getControlReference() {
		return this.controlRefrence;
	}

	/**
	 * Sets the current limit in Amps.
	 * 
	 * The motor controller will reduce the controller voltage output to avoid surpassing this limit. This limit is
	 * enabled by default and used for brushless only. This limit is highly recommended when using the NEO brushless
	 * motor.
	 * 
	 * The NEO Brushless Motor has a low internal resistance, which can mean large current spikes that could be enough
	 * to cause damage to the motor and controller. This current limit provides a smarter strategy to deal with high
	 * current draws and keep the motor and controller operating in a safe region.
	 * 
	 * @param limit - The current limit in Amps.
	 */
	public void setSmartCurrentLimit(int limit) {
		this.motor.setSmartCurrentLimit(limit);
	}
}
