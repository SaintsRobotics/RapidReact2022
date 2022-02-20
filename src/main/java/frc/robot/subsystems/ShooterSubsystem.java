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
	private final WPI_TalonFX m_shooterMotor = new WPI_TalonFX(ShooterConstants.kShooterMotorPort);

	private final SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(0.1, 0);
	private final PIDController m_shooterPID = new PIDController(0.0007, 0, 0);

	/** Creates a new {@link ShooterSubsystem}. */
	public ShooterSubsystem() {
		m_shooterPID.setSetpoint(0);
		m_shooterMotor.setInverted(true);
	}


	@Override
	public void periodic() {
		double m_velocityInRPM = Utils.ticksToRPM(m_shooterMotor.getSelectedSensorVelocity());
		double pidOutput = m_shooterPID.calculate(m_velocityInRPM);
		double feedForward = m_feedForward.calculate(m_shooterPID.getSetpoint());
		m_shooterMotor.set(pidOutput + feedForward);
		SmartDashboard.putNumber("Error", m_shooterPID.getPositionError());
		SmartDashboard.putNumber("PID output", pidOutput);
		SmartDashboard.putNumber("Feedforward", feedForward);
		SmartDashboard.putNumber("Current Shooter Power", m_shooterMotor.get());
		SmartDashboard.putNumber("Current Shooter Speed", m_velocityInRPM);
	}

	/**
	 * Sets the speed of the shooter.
	 * 
	 * @param speed Speed of the shooter in ticks per decisecond.
	 */
	public void set(double speed) {
		m_shooterPID.setSetpoint(speed);
		SmartDashboard.putNumber("Target Shooter Speed", speed);
	}
}
