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

  /**
   * Creates a new {@link ShooterSubsystem}.
   * 
   * @param shooterHardware The hardware for the {@link ShooterSubsystem}.
   */
  public ShooterSubsystem(ShooterHardware shooterHardware) {
    m_flywheelMotor = shooterHardware.shooter;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed", m_flywheelMotor.get());
  }

  /**
   * Sets the speed of the flywheel.
   * 
   * @param power The speed of the flywheel (-1 to 1).
   */
  public void set(double power) {
    m_flywheelMotor.set(power);
    SmartDashboard.putNumber("Desired Shooter Speed", power);
  }
}
