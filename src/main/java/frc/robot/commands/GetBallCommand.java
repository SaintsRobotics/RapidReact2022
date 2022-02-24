// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.subsystems.SwerveDriveSubsystem;

/** Uses the {@link Limelight} to intake a ball belonging to our alliance. */
public class GetBallCommand extends CommandBase {

	// TODO: tune pids
	private final PIDController m_xPID = new PIDController(0.03, 0, 0);
	private final PIDController m_yPID = new PIDController(0.03, 0, 0);
	private final PIDController m_rotPid = new PIDController(0.03, 0, 0);
	private MoveCommand m_moveCommand;
	private final int m_pipeline;

	/**
	 * Creates a new {@link GetBallCommand}.
	 * 
	 * @param subsystem The required subsystem.
	 * @param pipeline  Pipeline index to aim at the ball with.
	 */
	public GetBallCommand(MoveCommand moveCommand, int pipeline) {
		m_moveCommand = moveCommand;
		m_pipeline = pipeline;
		m_xPID.setTolerance(0.1);
		m_yPID.setTolerance(0.1);
		m_rotPid.setTolerance(0.1);
	}

	@Override
	public void initialize() {
		Limelight.setPipeline(m_pipeline);
		Limelight.setLED(0);
		Limelight.setCameraMode(0);

		m_moveCommand
				.withXSpeedSupplier(() -> m_xPID.calculate(
						NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0), 0))
				.withYSpeedSupplier(() -> m_yPID.calculate(
						NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0), 0))
				.withRotSpeedSupplier(() -> m_rotPid.calculate(Limelight.getX(), 0)).schedule();

		/**
		 * m_moveCommand .withXSpeedSupplier(() -> m_xPID.calculate(0, 0))
		 * .withYSpeedSupplier(() -> m_yPID.calculate(0, 0)) .withRotSpeedSupplier(() ->
		 * m_rotPid.calculate(Limelight.getX(), 0)).schedule();
		 */

	}

	@Override
	public void end(boolean interrupted) {
		Limelight.setLED(1);
		Limelight.setCameraMode(1);
		m_moveCommand.cancel();
	}

	// Change
	@Override
	public boolean isFinished() {
		return !Limelight.hasTarget();
	}
}
