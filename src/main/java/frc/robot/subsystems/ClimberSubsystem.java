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
import frc.robot.Constants;
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

	private double m_leftSpeed;
	private double m_rightSpeed;

	/** Creates a new {@link ClimberSubsystem}. */
	public ClimberSubsystem() {
		m_leftClimber.setInverted(ClimberConstants.kLeftArmReversed);
		m_rightClimber.setInverted(ClimberConstants.kRightArmReversed);
	}

	@Override
	public void periodic() {
		final double leftServoPosition = m_leftServo.get();
		final double rightServoPosition = m_rightServo.get();

		// Unlocks the servos before raising the left climber.
		m_leftServo.set(m_leftSpeed > 0 ? ClimberConstants.kLeftServoUnlockedPosition
				: ClimberConstants.kLeftServoLockedPosition);
		m_rightServo.set(m_rightSpeed > 0 ? ClimberConstants.kRightServoUnlockedPosition
				: ClimberConstants.kRightServoLockedPosition);

		if (m_leftEncoder.getAbsolutePosition() < Constants.ClimberConstants.kLeftArmMaxValue && m_leftEncoder.getAbsolutePosition() > Constants.ClimberConstants.kLeftArmMinValue) {
			m_leftClimber.set(m_leftSpeed);
		}
		else {
			m_leftClimber.set(0);
		}
		if (m_rightEncoder.getAbsolutePosition() < Constants.ClimberConstants.kRightArmMaxValue && m_rightEncoder.getAbsolutePosition() > Constants.ClimberConstants.kRightArmMinValue) {
			m_rightClimber.set(m_rightSpeed);
		} else {
			m_rightClimber.set(0);
		}

		if (OIConstants.kTelemetry) {
			SmartDashboard.putNumber("Climber Speed Desired Left", m_leftSpeed);
			SmartDashboard.putNumber("Climber Speed Desired Right", m_rightSpeed);
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
		m_leftSpeed = leftSpeed;
		m_rightSpeed = rightSpeed;
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
}
