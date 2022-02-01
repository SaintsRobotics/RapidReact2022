// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils;
import frc.robot.subsystems.ClimberArmSubsystem;
import frc.robot.Constants;

public class ClimberArmCommand extends CommandBase {

  private ClimberArmSubsystem m_climberSubsystem;
  private XboxController m_operatorController;

  /** Creates a new ClimberArmCommand. */
  public ClimberArmCommand(ClimberArmSubsystem climberSubsystem, XboxController operatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climberSubsystem = climberSubsystem;
    m_operatorController = operatorController;
    addRequirements(m_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberSubsystem.climb(0.2); //will add to constants later
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
