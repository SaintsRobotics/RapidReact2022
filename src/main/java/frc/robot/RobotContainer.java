// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public HardwareMap m_hardwareMap = new HardwareMap();
  private SwerveDriveSubsystem m_swerveDriveSubsystem = new SwerveDriveSubsystem(
      m_hardwareMap.swerveDrivetrainHardware);

  private XboxController m_driveController = new XboxController(OIConstants.kDriverControllerPort);

  private String trajectoryJSON = "output/Path.wpilib.json";
  private Trajectory trajectory = new Trajectory();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    DoubleSupplier x = () -> Utils
        .oddSquare(Utils.deadZone(-m_driveController.getLeftY(), OIConstants.kJoystickDeadzone))
        * SwerveConstants.kMaxSpeedMetersPerSecond * 0.2;
    DoubleSupplier y = () -> Utils
        .oddSquare(Utils.deadZone(-m_driveController.getLeftX(), OIConstants.kJoystickDeadzone))
        * SwerveConstants.kMaxSpeedMetersPerSecond * 0.2;
    DoubleSupplier rot = () -> Utils
        .oddSquare(Utils.deadZone(-m_driveController.getRightX(), OIConstants.kJoystickDeadzone))
        * SwerveConstants.kMaxAngularSpeedRadiansPerSecond * 0.2;
    m_swerveDriveSubsystem.setDefaultCommand(
        new SwerveDriveCommand(m_swerveDriveSubsystem, x, y, rot, () -> m_driveController.getRightBumper()));

    SmartDashboard.putNumber("Controller X", -m_driveController.getLeftY());
    SmartDashboard.putNumber("Controller Y", -m_driveController.getLeftX());
    SmartDashboard.putNumber("Controller Rot", -m_driveController.getRightY());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Resets the odometry when the back button is pressed.
    new JoystickButton(m_driveController, Button.kBack.value)
        .whenPressed(() -> m_swerveDriveSubsystem.resetOdometry(new Pose2d()), m_swerveDriveSubsystem);

    // Zeroes the heading when the start button is pressed
    new JoystickButton(m_driveController, Button.kStart.value).whenPressed(() -> m_swerveDriveSubsystem.zeroHeading(),
        m_swerveDriveSubsystem);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return pathFollowCommand();
  }

  public Command pathFollowCommand() {

    try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    PIDController xPID = new PIDController(Constants.SwerveConstants.kMaxSpeedMetersPerSecond, 0, 0);
    PIDController yPID = new PIDController(Constants.SwerveConstants.kMaxSpeedMetersPerSecond, 0, 0);
    ProfiledPIDController rotPID = new ProfiledPIDController(-Math.PI * 6, 0.0, 0.0,
                    new TrapezoidProfile.Constraints(
                                    Constants.SwerveConstants.kMaxAngularSpeedRadiansPerSecond,
                                    2.6));
    xPID.setTolerance(.05);
    yPID.setTolerance(0.05);
    rotPID.setTolerance(Math.PI / 24);
    rotPID.enableContinuousInput(-Math.PI, Math.PI);
    return new SwerveControllerCommand(trajectory, m_swerveDriveSubsystem::getCurrentPose,
                    m_swerveDriveSubsystem.getKinematics(), xPID, yPID, rotPID,
                    m_swerveDriveSubsystem::setSwerveModuleStates, m_swerveDriveSubsystem);

}
}
