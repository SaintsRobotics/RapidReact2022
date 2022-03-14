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
    double xPoseRobot = m_swerveSubsystem.getPose().getX();
    double yPoseRobot = m_swerveSubsystem.getPose().getY();

    double xHubDisplacement = Math.abs(xPoseRobot - Constants.FieldConstants.kHubX);
    double yHubDisplacement = Math.abs(yPoseRobot - Constants.FieldConstants.kHubY);
    
    double targetHeading;
    if(xPoseRobot > Constants.FieldConstants.kHubX && yPoseRobot > Constants.FieldConstants.kHubY) targetHeading = 180 + Math.toDegrees(Math.atan(xHubDisplacement/yHubDisplacement));
    else if(xPoseRobot < Constants.FieldConstants.kHubX && yPoseRobot > Constants.FieldConstants.kHubY) targetHeading = 180 - Math.toDegrees(Math.atan(xHubDisplacement/yHubDisplacement));
    else if(xPoseRobot < Constants.FieldConstants.kHubX && yPoseRobot < Constants.FieldConstants.kHubY) targetHeading = 90 - Math.toDegrees(Math.atan(xHubDisplacement/yHubDisplacement));
    else targetHeading = 360 - Math.toDegrees(Math.atan(xHubDisplacement/yHubDisplacement));
    m_moveCommand.withAbsoluteHeading(targetHeading);

    m_moveCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_moveCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
