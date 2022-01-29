// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;
import frc.robot.HardwareMap.SwerveModuleHardware;
import edu.wpi.first.math.geometry.Rotation2d;

/** Class that controls the swerve wheel and reads the swerve encoder. */
public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final CANCoder m_turningEncoder;

  private final PIDController m_turningPIDController = new PIDController(0.3, 0, 0);

  /**
   * Creates a new {@link SwerveModule}.
   * 
   * @param hardware       the hardware for the swerve module
   * @param driveMotor     motor that drives the wheel
   * @param turningMotor   motor that changes the angle of the wheel
   * @param turningEncoder absolute encoder for the swerve module
   */
  public SwerveModule(SwerveModuleHardware hardware, CANSparkMax driveMotor, CANSparkMax turningMotor,
      CANCoder turningEncoder) {
    m_driveMotor = driveMotor;
    m_turningMotor = turningMotor;

    m_turningEncoder = turningEncoder;

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
        / 60 / ModuleConstants.kDrivingGearRatio, Rotation2d.fromDegrees(m_turningEncoder.getPosition()));
  }

  /**
   * Stops the module from driving and turning. Use this so the wheels don't reset
   * to straight.
   */
  public void setDesiredState() {
    m_turningMotor.set(0);
    m_driveMotor.set(0);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getPosition() * Math.PI / 180));

    final double driveOutput = state.speedMetersPerSecond;
    final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getPosition() * Math.PI / 180,
        state.angle.getRadians());

    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }
}
