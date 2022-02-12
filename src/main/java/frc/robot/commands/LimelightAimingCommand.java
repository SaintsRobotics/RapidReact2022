// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;

/** Accesses {@link Limelight} values to aim the robot at a target. */
public class LimelightAimingCommand extends CommandBase {
  private final int m_pipeline;

  /**
   * Creates a new {@link LimelightAimingCommand}.
   * 
   * @param pipeline Index of the pipeline to use.
   */
  public LimelightAimingCommand(int pipeline) {
    m_pipeline = pipeline;
  }

  @Override
  public void initialize() {
    Limelight.setPipeline(m_pipeline);
    Limelight.setLed(0);
    SmartDashboard.putNumber("aiming command", 1);
  }

  @Override
  public void end(boolean interrupted) {
    Limelight.setLed(1);
    SmartDashboard.putNumber("aiming command", 0);
  }
}
