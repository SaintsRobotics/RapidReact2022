// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class TurnToHubCommand extends CommandBase {

  SwerveDriveSubsystem m_swerveSubsystem;
  MoveCommand m_moveCommand;
 
  public TurnToHubCommand(SwerveDriveSubsystem swerveSubsystem, MoveCommand moveCommand) {
    addRequirements(swerveSubsystem);
    m_moveCommand = moveCommand;
    m_swerveSubsystem = swerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double currentPose = m_swerveSubsystem.getPose().getRotation().getRadians();
    double currentX = m_swerveSubsystem.getPose().getX();
    double currentY = m_swerveSubsystem.getPose().getY();

    double xDistance = Math.abs(currentX - Constants.FieldConstants.cHubX);
    double yDistance = Math.abs(currentY - Constants.FieldConstants.cHubY);
    
    double theta = Math.atan(xDistance/yDistance);

    m_moveCommand.withAbsoluteHeading(360-theta);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
