// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.AbsoluteEncoder;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.HardwareMap.SwerveModuleHardware;

/** Class that controls the swerve wheel and reads the swerve encoder. */
public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final AbsoluteEncoder m_turningEncoder;

  // TODO Tune PIDs.
  static double Ku = 0.9;
  static double Tu = 0.38;
  private static final double f = 3*Ku*Tu/40;
  private final PIDController m_drivePIDController = new PIDController(0.6*Ku, 1.2*Ku/Tu, 0.03);
  private final PIDController m_turningPIDController = new PIDController(0.3, 0, 0);

  private SwerveModuleState m_state = new SwerveModuleState();

  /**
   * Creates a new {@link SwerveModule}.
   * 
   * @param hardware       the hardware for the swerve module
   * @param driveMotor     motor that drives the wheel
   * @param turningMotor   motor that changes the angle of the wheel
   * @param turningEncoder absolute encoder for the swerve module
   */
  public SwerveModule(SwerveModuleHardware hardware, CANSparkMax driveMotor, CANSparkMax turningMotor,
      AbsoluteEncoder turningEncoder) {
    m_driveMotor = driveMotor;
    m_turningMotor = turningMotor;

    m_turningEncoder = turningEncoder;

    m_drivePIDController.setIntegratorRange(0, 1);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_turningMotor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getEncoder().getVelocity() * ModuleConstants.kWheelCircumferenceMeters
        / 60 / ModuleConstants.kDrivingGearRatio, m_turningEncoder.get());
  }

  /**
   * Stops the module from driving and turning. Use this so the wheels don't reset
   * to straight.
   */
  public void setDesiredState() {
    m_turningMotor.set(0);
    m_driveMotor.set(0);

    m_state = new SwerveModuleState();
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState, double boost) {
    m_state = desiredState; // SwerveModuleState.optimize(desiredState, m_turningEncoder.get());

    final double driveOutput = m_drivePIDController.calculate(m_driveMotor.getEncoder().getVelocity()
        * ModuleConstants.kWheelCircumferenceMeters / 60 / ModuleConstants.kDrivingGearRatio,
        m_state.speedMetersPerSecond);
    final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.get().getRadians(),
        m_state.angle.getRadians());

    m_driveMotor.set(m_state.speedMetersPerSecond * boost / SwerveConstants.kMaxSpeedMetersPerSecond);
    m_turningMotor.set(turnOutput);
  }

  public SwerveModuleState getDesiredState() {
    return m_state;
  }
}
