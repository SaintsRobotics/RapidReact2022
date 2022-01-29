// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap.ShooterHardware;

public class shooterSubsystem extends SubsystemBase {
  /** Creates a new shooterSubsystem. */
  
  private MotorControllerGroup m_flywheelMotor; 
  private double m_targetSpeed;
  private double m_flywheelSpeed; 
  private TalonFX m_rightFlywheel;
  


  public shooterSubsystem(ShooterHardware shooterHardware) {
    m_flywheelMotor = shooterHardware.flywheel; 
    m_rightFlywheel = shooterHardware.rightFlywheel;
    
  }

  public void setFlywheelPower(double power) {
    this.m_targetSpeed = power;
  } 

  public double getFlywheelPower(){
    return m_rightFlywheel.getSelectedSensorVelocity();
    // not sure if this is correct way to get Velocity 
  }



  @Override
  public void periodic() {
    m_flywheelMotor.set(m_targetSpeed);
  }
}
