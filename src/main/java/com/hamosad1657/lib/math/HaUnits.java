
package com.hamosad1657.lib.math;

import edu.wpi.first.math.controller.PIDController;

public class HaUnits {
	public static final double kCANCoderTicksPerRev = 4096;
	public static double kChargedUpFieldLength = 16.5;

	public enum Velocity {
		kRPM, kMPS, kRadPS, kDegPS,
	}

	public enum Position {
		kRad, kDeg, kRot;
	}

	/**
	 * Represents a set of PID, feedforward and iZone values.
	 */
	public static class PIDGains {
		public double p, i, d, ff, iZone;

		/**
		 * @param p     - Proportional gain.
		 * @param i     - Integral gain.
		 * @param d     - Derivative gain.
		 * @param ff    - Feed Forward gain.
		 * @param iZone - If the absolute error is above iZone, the integral accumulator is cleared (making it
		 *              ineffective).
		 */
		public PIDGains(double p, double i, double d, double ff, double iZone) {
			this.p = p;
			this.i = i;
			this.d = d;
			this.ff = ff;
			this.iZone = iZone;
		}

		/**
		 * @param p  - Proportional gain.
		 * @param i  - Integral gain.
		 * @param d  - Derivative gain.
		 * @param ff - Feed Forward gain.
		 */
		public PIDGains(double p, double i, double d, double ff) {
			this.p = p;
			this.i = i;
			this.d = d;
			this.ff = ff;
			this.iZone = 0.0;
		}

		/**
		 * @param p - Proportional gain.
		 * @param i - Integral gain.
		 * @param d - Derivative gain.
		 */
		public PIDGains(double p, double i, double d) {
			this.p = p;
			this.i = i;
			this.d = d;
			this.ff = 0.0;
			this.iZone = 0.0;
		}

		public PIDGains() {
			this.p = 0.0;
			this.i = 0.0;
			this.d = 0.0;
			this.ff = 0.0;
			this.iZone = 0.0;
		}

		public PIDController toPIDController() {
			return new PIDController(this.p, this.i, this.d);
		}
	}

	public static double deadband(double value, double deadband) {
		if (Math.abs(value) > deadband) {
			return (value - deadband * Math.signum(value)) / (1.0 - deadband);
		} else {
			return 0.0;
		}
	}

	/**
	 * Gets a start range defined by [startMin] and [startMax] and an end range defined by [endMin] and [endMax], and a
	 * value that is relative to the first range.
	 * 
	 * @return The value relative to the end range.
	 */
	public static double mapRange(double value, double startMin, double startMax, double endMin, double endMax) {
		return endMin + ((endMax - endMin) / (startMax - startMin)) * (value - startMin);
	}
}
