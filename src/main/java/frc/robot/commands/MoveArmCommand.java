// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class MoveArmCommand extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;
  private XboxController m_controller;

  /** Creates a new MoveArmCommand. */
  public MoveArmCommand(IntakeSubsystem intakeSubsystem, XboxController controller) {
    m_intakeSubsystem = intakeSubsystem;
    addRequirements(m_intakeSubsystem);
    m_controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joystickOutput = m_controller.getRightY();
    if (joystickOutput < 0) { // if moving arm down 
      m_intakeSubsystem.moveArm(0.5 * joystickOutput);
   } else {
      m_intakeSubsystem.moveArm(joystickOutput);
    }
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
