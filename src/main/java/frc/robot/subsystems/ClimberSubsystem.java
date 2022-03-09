// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OIConstants;

/** Subsystem that controls the climber. */
public class ClimberSubsystem extends SubsystemBase {
	private final CANSparkMax m_leftClimber = new CANSparkMax(ClimberConstants.kLeftArmPort, MotorType.kBrushless);
	private final CANSparkMax m_rightClimber = new CANSparkMax(ClimberConstants.kRightArmPort, MotorType.kBrushless);

	private final Servo m_leftServo = new Servo(ClimberConstants.kLeftServoPort);
	private final Servo m_rightServo = new Servo(ClimberConstants.kRightServoPort);

	private final CANCoder m_leftEncoder = new CANCoder(ClimberConstants.kLeftEncoderPort);
	private final CANCoder m_rightEncoder = new CANCoder(ClimberConstants.kRightEncoderPort);

	private double m_leftClimbSpeed;
	private double m_rightClimbSpeed;

	/** Creates a new {@link ClimberSubsystem}. */
	public ClimberSubsystem() {
		m_leftClimber.setInverted(ClimberConstants.kLeftArmReversed);
		m_rightClimber.setInverted(ClimberConstants.kRightArmReversed);
	}

	@Override
	public void periodic() {
		final double leftServoPosition = m_leftServo.get();
		final double rightServoPosition = m_rightServo.get();

		// Unlocks the left servo before raising the left climber.
		if (m_leftClimbSpeed > 0) {
			m_leftServo.set(ClimberConstants.kLeftServoUnlockedPosition);

			// Checks that the left servo is released before running the motor by checking
			// if the position errors of the servo is within acceptable tolerance from the
			// unlocked position.
			m_leftClimber.set(
					atSetpoint(leftServoPosition, ClimberConstants.kLeftServoUnlockedPosition,
							ClimberConstants.kServoTolerance) ? m_leftClimbSpeed : 0);
		} else {
			m_leftServo.set(ClimberConstants.kLeftServoLockedPosition);
			m_leftClimber.set(m_leftClimbSpeed);
		}

		// Unlocks the right servo before raising the right climber.
		if (m_rightClimbSpeed > 0) {
			m_rightServo.set(ClimberConstants.kRightServoUnlockedPosition);

			// Checks that the right servo is released before running the motor by checking
			// if the position errors of the servo is within acceptable tolerance from the
			// unlocked position.
			m_rightClimber.set(
					atSetpoint(rightServoPosition, ClimberConstants.kRightServoUnlockedPosition,
							ClimberConstants.kServoTolerance) ? m_rightClimbSpeed : 0);
		} else {
			m_rightServo.set(ClimberConstants.kRightServoLockedPosition);
			m_rightClimber.set(m_rightClimbSpeed);
		}

		if (OIConstants.kTelemetry) {
			SmartDashboard.putNumber("Climber Speed Desired Left", m_leftClimbSpeed);
			SmartDashboard.putNumber("Climber Speed Desired Right", m_rightClimbSpeed);
			SmartDashboard.putNumber("Climber Speed Left", m_leftClimber.get());
			SmartDashboard.putNumber("Climber Speed Right", m_rightClimber.get());
			SmartDashboard.putNumber("Climber Servo Position Left", leftServoPosition);
			SmartDashboard.putNumber("Climber Servo Position Right", rightServoPosition);
			SmartDashboard.putNumber("Climber Position Left", m_leftEncoder.getPosition());
			SmartDashboard.putNumber("Climber Position Right", m_rightEncoder.getPosition());
		}
	}

	/**
	 * Sets the speed of the climber.
	 * 
	 * @param speed Speed from -1 to 1.
	 */
	public void setSpeed(double speed) {
		setSpeed(speed, speed);
	}

	/**
	 * Sets the speed of the climber.
	 * 
	 * @param leftSpeed  Speed from -1 to 1.
	 * @param rightSpeed Speed from -1 to 1.
	 */
	public void setSpeed(double leftSpeed, double rightSpeed) {
		m_leftClimbSpeed = leftSpeed;
		m_rightClimbSpeed = rightSpeed;
	}

	/**
	 * Returns the position of the left climber given by the left encoder.
	 * 
	 * @return The position of the left climber.
	 */
	public double getLeftPose() {
		return m_leftEncoder.getPosition();
	}

	/**
	 * Returns the position of the right climber given by the right encoder.
	 * 
	 * @return The position of the right climber.
	 */
	public double getRightPose() {
		return m_rightEncoder.getPosition();
	}

	/**
	 * Returns true if the measurement is within acceptable tolerance of the
	 * setpoint.
	 * 
	 * @param measurement The current measurement.
	 * @param setpoint    The desired measurement.
	 * @param tolerance   The acceptable tolerance.
	 * @return True if the measurement is within acceptable tolerance of the
	 *         setpoint.
	 */
	private boolean atSetpoint(double measurement, double setpoint, double tolerance) {
		return Math.abs(setpoint - measurement) < tolerance;
	}
}
