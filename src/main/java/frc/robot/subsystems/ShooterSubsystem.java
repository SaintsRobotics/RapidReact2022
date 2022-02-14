// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap.ShooterHardware;

/** Subsystem that controls the shooter. */
public class ShooterSubsystem extends SubsystemBase {
  private final WPI_TalonFX m_flywheelMotor;
  private double m_targetPower;

  /**
   * Creates a new {@link ShooterSubsystem}.
   * 
   * @param shooterHardware The hardware for the {@link ShooterSubsystem}.
   */
  public ShooterSubsystem(ShooterHardware shooterHardware) {
    m_flywheelMotor = shooterHardware.shooter;
    m_targetPower = 0;
  }

  public void setPower(double power) {
    m_targetPower = power;
  }

  /*
   * I was too lazy to make this a java doc, so this is what you may need to know
   * when making one
   * 4096 ticks per rotation
   * 60,000 ms in a min and m_flywheelMotor.getSelectedSensorVelocity() gives
   * ticks/100ms
   */
  public double getFlywheelRPM() {
    return m_flywheelMotor.getSelectedSensorVelocity() * 600 / 2048;
  }
  /*
   * Following is data (DO NOT DELETE):
   * at max power when copnnected to just a gear gives us 6450 +/- 100 (aaron at
   * 12.3V) (50%)
   * when connected to the shooter max rpm is 6010 +/- 10 (i actually paid
   * attenion and also aaron at 12.3V)
   * at 0.8 power 4715 +/- 10
   * at 0.67 power 3985 +/- 5
   * at 0.5 power 2980 +/- 5
   * at 0.33 power 1900 +/- 10
   * switching battery to ishaan at 12.9V (98%)
   * at 1 power 6280 +/- 10
   * at 0.8 power 4900 +/- 10
   * at 0.67 4160 +/- 10
   * at 0.5 3110 +/- 10
   * at 0.33 1980 +/- 10
   * switching to karthik at 33%
   * at 1 5935 +/- 10
   * at 0.8 4640 +/- 10
   * at 0.67 3935 +/- 15
   * at 0.5 2950 +/- 10
   * at 0.33 1875 +/- 10
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_flywheelMotor.set(m_targetPower);
    SmartDashboard.putNumber("Power", m_targetPower);
    SmartDashboard.putNumber("Target RPM", m_targetPower * 6380);
    SmartDashboard.putNumber("Current RPM", getFlywheelRPM());
  }

}
