// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.subsystems.SwerveDriveSubsystem;

/** Uses the {@link Limelight} to intake a ball belonging to our alliance. */
public class GetBallCommand extends CommandBase {
  private final SwerveDriveSubsystem m_swerveSubsystem;
  private final PIDController m_pid = new PIDController(0.03, 0, 0);
  private final int m_pipeline;

  /**
   * Creates a new {@link GetBallCommand}.
   * 
   * @param subsystem The required subsystem.
   * @param pipeline  Pipeline index to aim at the ball with.
   */
  public GetBallCommand(SwerveDriveSubsystem subsystem, int pipeline) {
    m_swerveSubsystem = subsystem;
    addRequirements(m_swerveSubsystem);

    m_pipeline = pipeline;

    m_pid.setTolerance(0.1);
  }

  @Override
  public void initialize() {
    Limelight.setPipeline(m_pipeline);
    Limelight.setLED(0);
    Limelight.setCameraMode(0);
  }

  @Override
  public void execute() {
    Limelight.setLED(0);
    m_swerveSubsystem.drive(m_pid.atSetpoint() ? 0.1 : 0, 0, m_pid.calculate(Limelight.getX(), 0), false);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.drive(0, 0, 0, false);
    Limelight.setLED(1);
  }

  // Change
  @Override
  public boolean isFinished() {
    return !Limelight.hasTarget();
  }
}
