// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Class for holding utility methods that do not apply to any specific command
 * or subsystem.
 */
public class Utils {
	/**
	 * Makes lower inputs smaller which allows for finer joystick control.
	 * 
	 * @param input The number to apply odd square to.
	 * @return The odd squared number.
	 */
	public static double oddSquare(double input) {
		return input * Math.abs(input);
	}

	/**
	 * Converts the speed of a TalonFX from the default units of ticks per
	 * decisecond to RPM.
	 * 
	 * @param ticksPerDecisecond The speed in ticks per decisecond.
	 * @return The speed in RPM.
	 */
	public static double toRPM(double ticksPerDecisecond) {
		return ticksPerDecisecond * 600 / 2048;
	}

	/**
	 * Returns whether a measurement is within acceptable tolerance of a setpoint
	 * 
	 * @param measurement The measurement value.
	 * @param setpoint    The desired value.
	 * @param tolerance   The acceptable deviation from the setpoint.
	 * @return Whether the measurement is within the acceptable tolerance the
	 *         setpoint.
	 */
	public static boolean atSetpoint(double measurement, double setpoint, double tolerance) {
		return Math.abs(setpoint - measurement) < tolerance;
	}
}
