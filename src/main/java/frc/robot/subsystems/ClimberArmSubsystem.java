// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberArmSubsystem extends SubsystemBase {
  private MotorControllerGroup m_climberMotor = new MotorControllerGroup(
      new CANSparkMax(ClimberConstants.kLeftArmPort, MotorType.kBrushless),
      new CANSparkMax(ClimberConstants.kRightArmPort, MotorType.kBrushless));

  /** Creates a new {@link ClimberArmSubsystem}. */
  public ClimberArmSubsystem() {
  }

  /**
   * Sets the speed of the climber.
   * 
   * @param speed Speed of the climber from -1 to 1.
   */
  public void set(double speed) {
    m_climberMotor.set(speed);
    SmartDashboard.putNumber("Desired Climber Speed", speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Speed", m_climberMotor.get());
  }
}
