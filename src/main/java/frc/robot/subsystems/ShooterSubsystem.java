// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

  @Override
  public void periodic() {
    m_shooterMotor.set(m_bangBangController.calculate(getFlywheelRPM()));

    SmartDashboard.putNumber("Current Shooter Speed", m_shooterMotor.get());
    SmartDashboard.putNumber("Current Shooter RPM", getFlywheelRPM());
  }

/**
 * Gets the RPM of the flywheel
 */
  public double getFlywheelRPM() {
    return m_shooterMotor.getSelectedSensorVelocity() * Constants.ShooterConstants.kMillisecondsPerMinute
        / Constants.ShooterConstants.kTicksPerRotation / Constants.ShooterConstants.kMillisecondsPerTenthSecond;
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
