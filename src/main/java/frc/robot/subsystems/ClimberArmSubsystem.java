 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareMap.ClimberArmHardware;

public class ClimberArmSubsystem extends SubsystemBase {
  private CANSparkMax m_right;
  private CANSparkMax m_left;
  /** Creates a new ClimberArmSubsystem. */
  public ClimberArmSubsystem(ClimberArmHardware hardware) {
    m_right = hardware.RightClimbingMotor;
    m_left = hardware.LeftClimbingMotor;
}
  public void climb(double m_desiredSpeed) {
    m_right.set(m_desiredSpeed);
    m_left.set(m_desiredSpeed);
  }

  public void unclimb() {
    m_right.set(0);
    m_left.set(0);
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}
