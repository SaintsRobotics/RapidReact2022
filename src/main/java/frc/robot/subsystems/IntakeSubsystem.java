// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax m_intakeController = new CANSparkMax(25, MotorType.kBrushless);
  private CANSparkMax m_armController = new CANSparkMax(24, MotorType.kBrushless);
  private CANSparkMax m_feederController = new CANSparkMax(23, MotorType.kBrushless);
  private double m_desiredInputSpeed;
  private double m_desiredFeederSpeed;

  /** Creates a new {@link IntakeSubsystem}. */
  public IntakeSubsystem() {
    m_armController.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_intakeController.set(m_desiredInputSpeed);
    m_feederController.set(m_desiredFeederSpeed);
  }

  public void turnFeederOn() {
    m_desiredFeederSpeed = 1;
  }

  public void turnFeederOff() {
    m_desiredFeederSpeed = 0;
  }

  public void moveArm(double velocity) {
    m_armController.set(velocity);
  }

  public void intake() {
    m_desiredInputSpeed = IntakeConstants.kIntakeSpeed;
  }

  public void outtake() {
    m_desiredInputSpeed = -IntakeConstants.kIntakeSpeed;
  }

  public void stopIntake() {
    m_desiredInputSpeed = 0;
  }

}
