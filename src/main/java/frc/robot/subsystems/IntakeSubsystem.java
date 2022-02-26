// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax m_armMotor = new CANSparkMax(24, MotorType.kBrushless);
  private final CANSparkMax m_intakeMotor = new CANSparkMax(25, MotorType.kBrushless);


  /** Creates a new {@link IntakeSubsystem}. */
  public IntakeSubsystem() {
    m_armMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Wheel Speed", m_intakeMotor.get());
    SmartDashboard.putNumber("Arm Motor Speed", m_armMotor.get());
  }

  /** Raises the arm. */
  public void raiseArm() {
    m_armMotor.set(IntakeConstants.kRaiseArmSpeed);
  }

  /** Lowers the arm. */
  public void lowerArm() {
    m_armMotor.set(IntakeConstants.kLowerArmSpeed);
  }


  /** Runs the intake. */
  public void intake() {
    m_intakeMotor.set(IntakeConstants.kIntakeSpeed);
  }

  /** Runs the intake in reverse. */
  public void intakeReverse() {
    m_intakeMotor.set(-IntakeConstants.kIntakeSpeed);
  }

  /** Turns off the intake. */
  public void intakeOff() {
    m_intakeMotor.set(0);
  }

}
