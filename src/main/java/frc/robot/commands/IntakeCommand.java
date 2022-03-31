
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

/** Runs the intake for a period of time. */
public class IntakeCommand extends CommandBase {
	private final ShooterSubsystem m_subsystem;
	private final Timer m_timer = new Timer();
	private double m_wait;

	/**
	 * Creates a new {@link IntakeCommand}.
	 * 
	 * @param subsystem The required subsystem.
	 */
	public IntakeCommand(ShooterSubsystem subsystem, double wait) {
		m_wait = wait;
		m_subsystem = subsystem;
		addRequirements(m_subsystem);
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
		return m_timer.get() > m_wait;
	}
}