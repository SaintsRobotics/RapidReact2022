// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.HardwareMap.IntakeHardware;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax m_intakeController;
  private CANSparkMax m_armController;
  private CANSparkMax m_feederController;
  private double m_desiredInputSpeed;
  private double m_desiredFeederSpeed;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(IntakeHardware intake) {
    m_intakeController = intake.intakeController;
    m_armController = intake.armController;
    m_feederController = intake.feederController;
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
    m_desiredInputSpeed = Constants.IntakeConstants.INTAKE_SPEED;
  }

  public void outtake() {
    m_desiredInputSpeed = -Constants.IntakeConstants.INTAKE_SPEED;
  }

  public void stopIntake() {
    m_desiredInputSpeed = 0;
  }

}
