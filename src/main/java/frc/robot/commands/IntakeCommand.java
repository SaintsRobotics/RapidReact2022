// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

/** Runs the intake for a period of time. */
public class IntakeCommand extends CommandBase {
	private final ShooterSubsystem m_subsystem;
	private final Timer m_timer = new Timer();
	private final double m_intakeTime;

	/**
	 * Creates a new {@link IntakeCommand}.
	 * 
	 * @param subsystem The required subsystem.
	 */
	public IntakeCommand(ShooterSubsystem subsystem, double time) {
		m_subsystem = subsystem;
		addRequirements(m_subsystem);
		m_intakeTime = time;
	}

	@Override
	public void initialize() {
		m_timer.reset();
		m_timer.start();
		m_subsystem.intake();
	}

	@Override
	public void end(boolean interrupted) {
		m_subsystem.intakeOff();
	}

	@Override
	public boolean isFinished() {
		return m_timer.get() > m_intakeTime;
	}
}
