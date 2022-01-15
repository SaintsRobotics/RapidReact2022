// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Utils;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Controls the {@link SwerveDriveSubsystem} using an {@link XboxController}.
 */
public class SwerveDriveCommand extends CommandBase {
  private final SwerveDriveSubsystem m_subsystem;
  private final XboxController m_xboxController;

  /**
   * Creates a new {@link SwerveDriveCommand}.
   * 
   * @param subsystem      the required subsystem
   * @param xboxController xbox controller for driving the robot
   */
  public SwerveDriveCommand(SwerveDriveSubsystem subsystem, XboxController xboxController) {
    m_subsystem = subsystem;
    m_xboxController = xboxController;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    double x = Utils.oddSquare(
        Utils.deadZone(m_xboxController.getLeftY() * SwerveConstants.kMaxSpeedMetersPerSecond, 0.25)) * 0.2;
    double y = Utils.oddSquare(
        Utils.deadZone(-m_xboxController.getLeftX() * SwerveConstants.kMaxSpeedMetersPerSecond, 0.25)) * 0.2;
    double rot = Utils.oddSquare(Utils.deadZone(
        -m_xboxController.getRightX() * SwerveConstants.kMaxAngularSpeedRadiansPerSecond, 0.25)) * 0.2;
    m_subsystem.drive(x, y, rot);

    SmartDashboard.putNumber("ControllerX", x);
    SmartDashboard.putNumber("ControllerY", y);
    SmartDashboard.putNumber("ControllerRot", rot);
  }
}
