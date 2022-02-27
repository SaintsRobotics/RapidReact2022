// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

/** Subsystem that controls the arm, intake, feeders, and shooter flywheel. */
public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax m_arm = new CANSparkMax(24, MotorType.kBrushless);
  private final DutyCycle m_armEncoder = new DutyCycle(new DigitalInput(0));
  private final CANSparkMax m_intake = new CANSparkMax(25, MotorType.kBrushless);
  private final MotorControllerGroup m_sideFeeders;
  private final CANSparkMax m_topFeeder = new CANSparkMax(23, MotorType.kBrushless);
  private final WPI_TalonFX m_shooter = new WPI_TalonFX(ShooterConstants.kShooterMotorPort);
  private final BangBangController m_shooterController = new BangBangController();

  /** Creates a new {@link ShooterSubsystem}. */
  public ShooterSubsystem() {
    m_arm.setIdleMode(IdleMode.kBrake);
    CANSparkMax leftFeeder = new CANSparkMax(22, MotorType.kBrushless);
    CANSparkMax rightFeeder = new CANSparkMax(21, MotorType.kBrushless);
    rightFeeder.setInverted(true);
    m_sideFeeders = new MotorControllerGroup(leftFeeder, rightFeeder);
  }

  // top feeder run for how long?
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_shooter.set(m_shooterController.getSetpoint() == 0 ? 0
        : m_shooterController.calculate(m_shooter.getSelectedSensorVelocity()));

    SmartDashboard.putNumber("Current Shooter Power", m_shooter.get());
    SmartDashboard.putNumber("Current Shooter Speed", m_shooter.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Intake Wheel Speed", m_intake.get());
    SmartDashboard.putNumber("Arm Motor Speed", m_arm.get());
    SmartDashboard.putNumber("DutyCycle", getAbsolutePosition());
  }

  /** Raises the arm. */
  public void raiseArm() {
    m_arm.set(IntakeConstants.kRaiseArmSpeed);
  }

  /** Lowers the arm. */
  public void lowerArm() {
    m_arm.set(IntakeConstants.kLowerArmSpeed);
  }

  /** Runs the intake. */
  public void intake() {
    m_intake.set(IntakeConstants.kIntakeSpeed);
  }

  /** Runs the intake in reverse. */
  public void intakeReverse() {
    m_intake.set(-IntakeConstants.kIntakeSpeed);
  }

  /** Turns off the intake. */
  public void intakeOff() {
    m_intake.set(0);
  }

  public void setSideFeederSpeed(double speed) {
    m_intake.set(IntakeConstants.kFeederSpeed);
  }

  public void setTopFeederSpeed() {
    m_intake.set(IntakeConstants.kFeederSpeed);
  }

  public void setLowerArmSpeed() {
    m_arm.set(IntakeConstants.kLowerArmSpeed);
  }

  public void setRaiseArmSpeed() {
    m_arm.set(IntakeConstants.kRaiseArmSpeed);
  }

  /**
   * Get the absolute position of the duty cycle encoder.
   *
   * <p>
   * getAbsolutePosition() - getPositionOffset() will give an encoder absolute
   * position relative
   * to the last reset. 
   *
   * <p>
   * This will not account for rollovers, and will always be just the raw absolute
   * position.
   * 
   * <p>
   * Ranges from 0 degrees to 360 degrees
   *
   * @return the absolute position
   */
  public double getAbsolutePosition() {
    return m_armEncoder.getOutput() * 360;
  }

  /**
   * Sets the speed of the shooter.
   * 
   * @param speed Speed of the shooter in ticks per decisecond.
   */
  public void setShooterSpeed(double speed) {
    m_shooterController.setSetpoint(speed);
    SmartDashboard.putNumber("Target Shooter Speed", speed);
  }

}
