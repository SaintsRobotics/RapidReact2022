// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * Subsystem that controls the RPM of the shooter by using a bang bang
 * controller.
 */
public class ShooterSubsystem extends SubsystemBase {
  private final WPI_TalonFX m_shooterMotor = new WPI_TalonFX(ShooterConstants.kShooterMotorPort);

  private final BangBangController m_bangBangController = new BangBangController();

  /** Creates a new {@link ShooterSubsystem}. */
  public ShooterSubsystem() {
    // TODO check if it is set to coast by default (if so delete line below)
    m_shooterMotor.setNeutralMode(NeutralMode.Coast);
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
    m_shooterMotor.set(m_bangBangController.calculate(getFlywheelRPM()));

    SmartDashboard.putNumber("Current Shooter Speed", m_shooterMotor.get());
    SmartDashboard.putNumber("Current Shooter RPM", getFlywheelRPM());
  }

  /*
   * I was too lazy to make this a java doc, so this is what you may need to know
   * when making one
   * 4096 ticks per rotation
   * 60,000 ms in a min and m_shooterMotor.getSelectedSensorVelocity() gives
   * ticks/100ms
   */
  public double getFlywheelRPM() {
    return m_shooterMotor.getSelectedSensorVelocity() * 600 / 2048;
  }

  /**
   * Sets the RPM of the shooter.
   * 
   * @param revs RPM of the shooter.
   */
  public void set(double revs) {
    m_bangBangController.setSetpoint(revs);
    SmartDashboard.putNumber("Target Shooter RPM", revs);
  }
}
