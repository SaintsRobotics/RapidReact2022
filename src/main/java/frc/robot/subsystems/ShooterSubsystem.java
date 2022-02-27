// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.DutyCycleAbsoluteEncoder;

/** Subsystem that controls the arm, intake, feeders, and shooter flywheel. */
public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax m_arm = new CANSparkMax(ShooterConstants.kArmPort, MotorType.kBrushless);
  private final DutyCycleAbsoluteEncoder m_armEncoder = new DutyCycleAbsoluteEncoder(0);
  private final CANSparkMax m_intake = new CANSparkMax(ShooterConstants.kIntakeWheelsPort, MotorType.kBrushless);
  private final MotorControllerGroup m_sideFeeders;
  private final CANSparkMax m_topFeeder = new CANSparkMax(ShooterConstants.kTopFeederPort, MotorType.kBrushless);
  private final WPI_TalonFX m_flywheel = new WPI_TalonFX(ShooterConstants.kFlywheelPort);
  private final BangBangController m_shooterController = new BangBangController();

  private final PIDController m_PID = new PIDController(0.3, 0, 0);
  /** Creates a new {@link ShooterSubsystem}. */
  public ShooterSubsystem() {
    m_arm.setIdleMode(IdleMode.kBrake);
    CANSparkMax leftFeeder = new CANSparkMax(ShooterConstants.kLeftFeederPort, MotorType.kBrushless);
    CANSparkMax rightFeeder = new CANSparkMax(ShooterConstants.kRightFeederPort, MotorType.kBrushless);
    rightFeeder.setInverted(true);
    m_sideFeeders = new MotorControllerGroup(leftFeeder, rightFeeder);
  }

  // top feeder run for how long?
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_flywheel.set(m_shooterController.getSetpoint() == 0 ? 0
        : m_shooterController.calculate(m_flywheel.getSelectedSensorVelocity()));

    SmartDashboard.putNumber("Current Shooter Power", m_flywheel.get());
    SmartDashboard.putNumber("Current Shooter Speed", m_flywheel.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Intake Wheel Speed", m_intake.get());
    SmartDashboard.putNumber("Arm Motor Speed", m_arm.get());
    SmartDashboard.putNumber("DutyCycle", m_armEncoder.getAbsolutePosition());
  }

  /** Raises the arm. */
  public void raiseArm() {
    m_arm.set(ShooterConstants.kRaiseArmSpeed);
  }

  /** Lowers the arm. */
  public void lowerArm() {
    m_arm.set(ShooterConstants.kLowerArmSpeed);
  }

  /** Runs the intake. */
  public void intake() {
    m_intake.set(ShooterConstants.kIntakeSpeed);
  }

  /** Runs the intake in reverse. */
  public void intakeReverse() {
    m_intake.set(-ShooterConstants.kIntakeSpeed);
  }

  /** Turns off the intake. */
  public void intakeOff() {
    m_intake.set(0);
  }

  public void setSideFeederSpeed() {
    m_sideFeeders.set(ShooterConstants.kFeederSpeed);
  }

  public void setTopFeederSpeed() {
    m_topFeeder.set(ShooterConstants.kFeederSpeed);
  }

  public void setLowerArmSpeed() {
    m_arm.set(ShooterConstants.kLowerArmSpeed);
  }

  public void setRaiseArmSpeed() {
    m_arm.set(ShooterConstants.kRaiseArmSpeed);
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
