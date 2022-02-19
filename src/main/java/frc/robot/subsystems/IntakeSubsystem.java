// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax m_armMotor = new CANSparkMax(24, MotorType.kBrushless);
  private final CANSparkMax m_intakeMotor = new CANSparkMax(25, MotorType.kBrushless);
  private final CANSparkMax m_feederMotor = new CANSparkMax(23, MotorType.kBrushless);

  // TODO tune PID
  private final PIDController m_armPID = new PIDController(0.3, 0, 0);

  private double m_desiredInputSpeed;
  private double m_desiredFeederSpeed;

  /** Creates a new {@link IntakeSubsystem}. */
  public IntakeSubsystem() {
    m_armMotor.setIdleMode(IdleMode.kBrake);
    m_armPID.setSetpoint(IntakeConstants.kRaisedArmAngle);
  }

  @Override
  public void periodic() {
    // TODO use encoder to determine arm position
    m_armMotor.set(m_armPID.calculate(0));
    m_intakeMotor.set(m_desiredInputSpeed);
    m_feederMotor.set(m_desiredFeederSpeed);
  }

  /** Raises the arm. */
  public void raiseArm() {
    m_armPID.setSetpoint(IntakeConstants.kRaisedArmAngle);
  }

  /** Lowers the arm. */
  public void lowerArm() {
    m_armPID.setSetpoint(IntakeConstants.kLoweredArmAngle);
  }

  public void turnFeederOn() {
    m_desiredFeederSpeed = 1;
  }

  public void turnFeederOff() {
    m_desiredFeederSpeed = 0;
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
