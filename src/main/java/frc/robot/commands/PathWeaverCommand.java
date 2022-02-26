// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

/** Follows a {@link Trajectory} created by PathWeaver. */
public class PathWeaverCommand extends CommandBase {
	private final SwerveDriveSubsystem m_subsystem;

	private final PIDController m_xPID = new PIDController(1, 0, 0);
	private final PIDController m_yPID = new PIDController(1, 0, 0);
	private final ProfiledPIDController m_rotPID = new ProfiledPIDController(1, 0, 0,
			new TrapezoidProfile.Constraints(Constants.SwerveConstants.kMaxAngularSpeedRadiansPerSecond, 2.6));
	private final Boolean m_resetOdometry;

	// Defaults to an empty trajectory if PathWeaver file can not be found.
	private Trajectory m_trajectory = new Trajectory();
	private SwerveControllerCommand m_command;

	/**
	 * Creates a new {@link PathWeaverCommand} that follows a path generated by
	 * PathWeaver.
	 * 
	 * @param subsystem      The required subsystem.
	 * @param trajectoryJSON The name of the json file containing the path. In the
	 *                       format "paths/trajectory.wpilib.json". Should be stored
	 *                       under deploy>paths.
	 * @param resetOdometry  Whether to reset odometry to the starting position of
	 *                       the path. Should be true if it is the first path in a
	 *                       series of paths.
	 */
	public PathWeaverCommand(SwerveDriveSubsystem subsystem, String trajectoryJSON, Boolean resetOdometry) {
		m_subsystem = subsystem;
		addRequirements(m_subsystem);

		m_resetOdometry = resetOdometry;

		m_xPID.setTolerance(0.05);
		m_yPID.setTolerance(0.05);
		m_rotPID.setTolerance(Math.PI / 24);
		m_rotPID.enableContinuousInput(-Math.PI, Math.PI);

		try {
			m_trajectory = TrajectoryUtil
					.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON));
		} catch (IOException ex) {
			DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
		}
	}

	@Override
	public void initialize() {
		if (m_resetOdometry) {
			m_subsystem.resetOdometry(m_trajectory.getInitialPose());
		}
		m_command = new SwerveControllerCommand(
				m_trajectory,
				m_subsystem::getPose,
				SwerveConstants.kDriveKinematics,
				m_xPID,
				m_yPID,
				m_rotPID,
				m_subsystem::setModuleStates,
				m_subsystem);
		m_command.schedule();
	}

	@Override
	public void end(boolean interrupted) {
		m_command.end(interrupted);
		m_subsystem.drive(0, 0, 0, false);
	}

	@Override
	public boolean isFinished() {
		return m_command.isFinished();
	}
}
