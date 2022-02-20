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

	private final PIDController m_PID = new PIDController(ShooterConstants.kP, 0, 0);
	private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(ShooterConstants.kS, 0);

	/** Creates a new {@link ShooterSubsystem}. */
	public ShooterSubsystem() {
		m_shooterMotor.setInverted(true);
		m_PID.setSetpoint(0);
	}

	@Override
	public void periodic() {
		double pidOutput = m_PID.calculate(Utils.ticksToRPM(m_shooterMotor.getSelectedSensorVelocity()));
		double feedforward = m_feedforward.calculate(m_PID.getSetpoint());
		m_shooterMotor.set(pidOutput + feedforward);

		SmartDashboard.putNumber("PID Output", pidOutput);
		SmartDashboard.putNumber("Feedforward", feedforward);
		SmartDashboard.putNumber("Current Shooter Speed (-1 to 1)", m_shooterMotor.get());
		SmartDashboard.putNumber("Current Shooter RPM", Utils.ticksToRPM(m_shooterMotor.getSelectedSensorVelocity()));
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
