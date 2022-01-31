// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.HardwareMap.SwerveModuleHardware;

/** Class that controls the swerve wheel and reads the swerve encoder. */
public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final CANCoder m_turningEncoder;

  private final PIDController m_turningPIDController = new PIDController(0.3, 0, 0);

  /**
   * Creates a new {@link SwerveModule}.
   * 
   * @param hardware             The hardware for the swerve module.
   * @param driveMotor           Motor that drives the wheel.
   * @param driveMotorReversed   Whether the drive motor is reversed.
   * @param turningMotor         Motor that changes the angle of the wheel.
   * @param turningEncoder       Absolute encoder for the swerve module.
   * @param turningEncoderOffset Offset of the turning encoder in degrees.
   */
  public SwerveModule(
      SwerveModuleHardware hardware,
      CANSparkMax driveMotor,
      boolean driveMotorReversed,
      CANSparkMax turningMotor,
      CANCoder turningEncoder,
      double turningEncoderOffset) {
    m_driveMotor = driveMotor;
    m_turningMotor = turningMotor;
    m_turningEncoder = turningEncoder;

    // converts default units to meters per second
    m_driveMotor.getEncoder().setVelocityConversionFactor(
        ModuleConstants.kWheelCircumferenceMeters / 60 / ModuleConstants.kDrivingGearRatio);
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_driveMotor.setInverted(driveMotorReversed);

    m_turningMotor.setIdleMode(IdleMode.kBrake);

    // converts default units to radians
    m_turningEncoder.configFeedbackCoefficient(Math.toRadians(0.087890625), "radians", SensorTimeBase.PerSecond);
    m_turningEncoder.configMagnetOffset(turningEncoderOffset);

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getEncoder().getVelocity(),
        new Rotation2d(m_turningEncoder.getAbsolutePosition()));
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
    SwerveModuleState state = SwerveModuleState.optimize(desiredState,
        new Rotation2d(m_turningEncoder.getAbsolutePosition()));

    final double driveOutput = state.speedMetersPerSecond / SwerveConstants.kMaxSpeedMetersPerSecond;
    final double turnOutput = m_turningPIDController.calculate(m_turningEncoder.getAbsolutePosition(),
        state.angle.getRadians());

    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }
}
