// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap.ShooterHardware;

public class shooterSubsystem extends SubsystemBase {
  /** Creates a new shooterSubsystem. */

  private WPI_TalonFX m_flywheelMotor;
  private double m_targetSpeed;
  private double m_flywheelSpeed;

  public shooterSubsystem(ShooterHardware shooterHardware) {
    m_flywheelMotor = shooterHardware.flywheel;

  }

  public void setFlywheelPower(double power) {
    this.m_targetSpeed = power;
  }

  public double getFlywheelPower() {
    return m_flywheelMotor.get();
  }

  @Override
  public void periodic() {
    m_flywheelMotor.set(m_targetSpeed);
  }
}
