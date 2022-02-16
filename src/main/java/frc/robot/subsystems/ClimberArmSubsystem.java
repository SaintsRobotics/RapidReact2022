// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberArmSubsystem extends SubsystemBase {
  private CANSparkMax m_right = new CANSparkMax(ClimberConstants.kRightArmPort, MotorType.kBrushless);
  private CANSparkMax m_left = new CANSparkMax(ClimberConstants.kLeftArmPort, MotorType.kBrushless);

  /** Creates a new ClimberArmSubsystem. */
  public ClimberArmSubsystem() {
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

  }
}
