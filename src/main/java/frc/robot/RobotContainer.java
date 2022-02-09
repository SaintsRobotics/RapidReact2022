// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.LimelightAimingCommand;
import frc.robot.commands.MoveCommand;
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

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();
        DoubleSupplier x = () -> Utils
                .oddSquare(Utils.deadZone(-m_driveController.getLeftY(), OIConstants.kJoystickDeadzone))
                * SwerveConstants.kMaxSpeedMetersPerSecond;
        DoubleSupplier y = () -> Utils
                .oddSquare(Utils.deadZone(-m_driveController.getLeftX(), OIConstants.kJoystickDeadzone))
                * SwerveConstants.kMaxSpeedMetersPerSecond;
        DoubleSupplier rot = () -> Utils
                .oddSquare(Utils.deadZone(-m_driveController.getRightX(), OIConstants.kJoystickDeadzone))
                * SwerveConstants.kMaxAngularSpeedRadiansPerSecond;
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
        new JoystickButton(m_driveController, Button.kStart.value).whenPressed(
                () -> m_swerveDriveSubsystem.zeroHeading(),
                m_swerveDriveSubsystem);

        // Aims at target while the A button is held.
        new JoystickButton(m_driveController, Button.kA.value)
                .whenHeld(new LimelightAimingCommand(m_swerveDriveSubsystem, 0));

        // Aims at blue balls while the B button is held.
        new JoystickButton(m_driveController, Button.kB.value)
                .whenHeld(new LimelightAimingCommand(m_swerveDriveSubsystem, 1));

        // Aims at red balls while the X button is held.
        new JoystickButton(m_driveController, Button.kX.value)
                .whenHeld(new LimelightAimingCommand(m_swerveDriveSubsystem, 2));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        /*
         * The plan for two ball autonomous mode:
         * Starting Position: facing towards the ball, at the corner parallel to the
         * line
         * 1. Go to ball (drive forwards 1 meter)
         * 2. Run intake for 1 second
         * 2. Turn around and get into position for shooting the ball (drive field
         * relative backwards 3 meters, turn 180-ish degrees)
         * 3. Shoot the ball
         */
        
        // NOTE: ADD INTAKE COMMAND AND SHOOT COMMAND TO THE SEQUENCE
        return new SequentialCommandGroup(new MoveCommand(m_swerveDriveSubsystem).withRobotRelativeX(1),
                new ParallelCommandGroup(
                        new MoveCommand(m_swerveDriveSubsystem).withFieldRelativeX(-3).withChangeInHeading(180),
                        new LimelightAimingCommand(m_swerveDriveSubsystem, 0)));
    }
}
