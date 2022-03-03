// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils;
import frc.robot.Constants.ShooterConstants;

/**
 * Subsystem that controls the RPM of the shooter by using a PID controller with
 * feed forward.
 */
public class ShooterSubsystem extends SubsystemBase {
	private final WPI_TalonFX m_motor = new WPI_TalonFX(ShooterConstants.kShooterMotorPort);

	private final PIDController m_PID = new PIDController(0.0007, 0, 0);
	private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.6, 0);

	/** Creates a new {@link ShooterSubsystem}. */
	public ShooterSubsystem() {
		m_motor.setInverted(true);
		m_PID.setSetpoint(0);
	}

	@Override
	public void periodic() {
		double pidOutput = m_PID.calculate(Utils.toRPM(m_motor.getSelectedSensorVelocity()));
		m_motor.set(pidOutput + m_feedforward.calculate(m_PID.getSetpoint()));

		SmartDashboard.putNumber("PID Output", pidOutput);
		SmartDashboard.putNumber("Feedforward", m_feedforward.calculate(m_PID.getSetpoint()));
		SmartDashboard.putNumber("Current Shooter Speed (-1 to 1)", m_motor.get());
		SmartDashboard.putNumber("Current Shooter RPM", Utils.toRPM(m_motor.getSelectedSensorVelocity()));
	}

	/**
	 * Sets the speed of the shooter.
	 * 
	 * @param speed Speed of the shooter in RPM.
	 */
	public void set(double speed) {
		m_PID.setSetpoint(speed);
		SmartDashboard.putNumber("Target Shooter RPM", speed);
	}
}
