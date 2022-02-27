// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;

/** Uses the {@link Limelight} to intake a ball belonging to our alliance. */
public class GetBallCommand extends CommandBase {

	// TODO: tune pid
	private final PIDController m_rotPid = new PIDController(0.03, 0, 0);
	private MoveCommand m_moveCommand;
	private final int m_pipeline;
	private static double COLLECTION_TOLERANCE = 0.45;
	private static double MOUNTING_ANGLE_DEGREES = -10;
	private static double LIMELIGHT_HEIGHT = 0.5;
	private static double BALL_CENTER_HEIGHT = 0.1;

	/**
	 * Creates a new {@link GetBallCommand}.
	 * 
	 * @param subsystem The required subsystem.
	 * @param pipeline  Pipeline index to aim at the ball with.
	 */
	public GetBallCommand(MoveCommand moveCommand, int pipeline) {
		m_moveCommand = moveCommand;
		m_pipeline = pipeline;
		m_rotPid.setTolerance(0.1);
	}

	@Override
	public void initialize() {
		Limelight.setPipeline(m_pipeline);
		Limelight.setLED(0);
		Limelight.setCameraMode(0);

		double distance = getDistance();

		m_moveCommand.withRotSpeedSupplier(() -> m_rotPid.calculate(Limelight.getX(), 0)).withRobotRelativeX(distance)
				.schedule();
	}

	@Override
	public void end(boolean interrupted) {
		Limelight.setLED(1);
		Limelight.setCameraMode(1);
		m_moveCommand.cancel();
	}

	@Override
	public boolean isFinished() {
		return Math.abs(getDistance()) < COLLECTION_TOLERANCE
				&& Math.abs(m_rotPid.calculate(Limelight.getX(), 0)) < 0.03;
	}

	public double getDistance() {
		return (BALL_CENTER_HEIGHT - LIMELIGHT_HEIGHT)
				/ (Math.tan((MOUNTING_ANGLE_DEGREES + Limelight.getY()) * Math.PI / 180)); // units in
		// meters,
		// converted
		// from degrees
		// to radians
	}
}
