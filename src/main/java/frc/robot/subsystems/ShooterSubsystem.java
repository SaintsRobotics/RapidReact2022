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

  public void setPower(double power){
    m_targetPower = power;
  }

  public double getFlywheelRPM(){
    return m_flywheelMotor.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_flywheelMotor.set(m_targetPower);
    SmartDashboard.putNumber("Power", m_targetPower);
    SmartDashboard.putNumber("Target RPM", m_targetPower*6380);
    SmartDashboard.putNumber("Current RPM", getFlywheelRPM());
  }

}
