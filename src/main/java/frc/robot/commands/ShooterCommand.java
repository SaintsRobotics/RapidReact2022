// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SShooterSubsystem;

public class ShooterCommand extends CommandBase {

  private SShooterSubsystem m_shooterSubsystem; 
  private PIDController m_pidController = new PIDController(0.7, 0, 0);

  /** Creates a new ShooterCommand. */
  public ShooterCommand(SShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSubsystem = shooterSubsystem;
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    m_pidController.setSetpoint(0.67);
    SmartDashboard.putNumber("Shooter speed:", m_shooterSubsystem.m_flywheelMotor.getMotorOutputPercent());
    m_shooterSubsystem.setFlywheelPower(m_pidController.calculate(m_shooterSubsystem.getFlywheelPower()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.setFlywheelPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
