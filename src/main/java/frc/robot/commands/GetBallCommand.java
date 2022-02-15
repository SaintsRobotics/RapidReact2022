// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Limelight;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class GetBallCommand extends CommandBase {
  private final PIDController m_pid = new PIDController(0.03, 0, 0);
  private SwerveDriveSubsystem m_swerveSubsystem;
  /** Creates a new AutonAimingCommand. */

  public void AutonAimingCommand(SwerveDriveSubsystem swerveSubsystem) {
    m_swerveSubsystem = swerveSubsystem;
    addRequirements(m_swerveSubsystem);
  
  }
    // Use addRequirements() here to declare subsystem dependencies.
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pid.reset();
    Limelight.setLED(3);
    m_pid.setSetpoint(0.0); // 0.0 means the limelight is pointed at the right direction
    m_pid.setTolerance(Math.PI/30);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Ball Resolved", Limelight.getState());
    m_swerveSubsystem.drive(m_pid.calculate(Limelight.getX()), m_pid.calculate(Limelight.getY()), 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveSubsystem.drive(0, 0, 0, true);
    Limelight.setLED(1);
  }

}