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
   * If input is within tolerance of desiredVal, it returns desiredVal. If
   * input is outside the tolerance, then it returns input.
   * 
   * @param input      The input number.
   * @param desiredVal The desired value.
   * @param tolerance  The tolerance.
   */
  public static double tolerance(double input, double desiredVal, double tolerance) {
    if (Math.abs(input - desiredVal) < tolerance) {
      return desiredVal;
    }
    return input;
  }

  /**
   * Makes lower inputs smaller which allows for finer joystick control.
   * 
   * @param input The number to apply odd square to.
   * @return The odd squared number.
   */
  public static double oddSquare(double input) {
    return input * Math.abs(input);
  }
}
