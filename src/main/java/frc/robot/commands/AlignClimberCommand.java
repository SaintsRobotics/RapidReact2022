// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

/** Aligns both the arms of the climber. */
public class AlignClimberCommand extends CommandBase {
  private final ClimberSubsystem m_subsystem;

  private final PIDController m_leftPID = new PIDController(0.3, 0, 0);
  private final PIDController m_rightPID = new PIDController(0.3, 0, 0);

  /**
   * Creates a new {@link AlignClimberCommand}.
   * 
   * @param subsystem The required subsystem.
   */
  public AlignClimberCommand(ClimberSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);

    m_rightPID.setTolerance(0.1);
    m_leftPID.setTolerance(0.1);
  }

  @Override
  public void initialize() {
    final double center = (m_subsystem.getLeftPose() + m_subsystem.getRightPose()) / 2;
    m_leftPID.setSetpoint(center);
    m_rightPID.setSetpoint(center);
  }

  @Override
  public void execute() {
    m_subsystem.setSpeed(
        m_leftPID.calculate(m_subsystem.getLeftPose()),
        m_rightPID.calculate(m_subsystem.getRightPose()));
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return m_leftPID.atSetpoint() && m_rightPID.atSetpoint();
  }
}
