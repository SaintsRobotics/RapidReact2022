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
import frc.robot.Utils;

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
		m_leftEncoder.setPosition(0);
		m_rightEncoder.setPosition(0);

	}

	@Override
	public void periodic() {
		// Unlocks the servos before raising the left climber.
		m_leftServo.set(m_leftSpeed > 0 ? ClimberConstants.kLeftServoUnlockedPosition
				: ClimberConstants.kLeftServoLockedPosition);
		m_rightServo.set(m_rightSpeed > 0 ? ClimberConstants.kRightServoUnlockedPosition
				: ClimberConstants.kRightServoLockedPosition);

		m_leftClimber.set(m_leftSpeed);
		m_rightClimber.set(m_rightSpeed);
		// if (m_leftEncoder.getAbsolutePosition() <
		// Constants.ClimberConstants.kLeftArmMaxValue &&
		// m_leftEncoder.getAbsolutePosition() >
		// Constants.ClimberConstants.kLeftArmMinValue) {
		// m_leftClimber.set(m_leftSpeed);
		// }
		// else {
		// m_leftClimber.set(0);
		// }
		// if (m_rightEncoder.getAbsolutePosition() <
		// Constants.ClimberConstants.kRightArmMaxValue &&
		// m_rightEncoder.getAbsolutePosition() >
		// Constants.ClimberConstants.kRightArmMinValue) {
		// m_rightClimber.set(m_rightSpeed);
		// } else {
		// m_rightClimber.set(0);
		// }

		SmartDashboard.putNumber("Climber Position Left", m_leftEncoder.getPosition());
		SmartDashboard.putNumber("Climber Position Right", m_rightEncoder.getPosition());

		if (OIConstants.kTelemetry) {
			SmartDashboard.putNumber("Climber Power Left", m_leftClimber.get());
			SmartDashboard.putNumber("Climber Power Right", m_rightClimber.get());
			SmartDashboard.putNumber("Climber Servo Position Left", m_leftServo.get());
			SmartDashboard.putNumber("Climber Servo Position Right", m_rightServo.get());
		}

		SmartDashboard.putNumber("Temperature Climber Left", m_leftClimber.getMotorTemperature());
		SmartDashboard.putNumber("Temperature Climber Right", m_rightClimber.getMotorTemperature());

		SmartDashboard.putNumber("Current Climber Left", m_leftClimber.getOutputCurrent());
		SmartDashboard.putNumber("Current Climber Right", m_rightClimber.getOutputCurrent());
	}

	/**
	 * Sets the speed of the climber.
	 * 
	 * @param speed Speed from -1 to 1.
	 */
	public void set(double speed) {
		set(speed, speed);
	}

	/**
	 * Sets the speed of the climber.
	 * 
	 * @param leftSpeed  Speed from -1 to 1.
	 * @param rightSpeed Speed from -1 to 1.
	 */
	public void set(double leftSpeed, double rightSpeed) {
		// Unlocks the servos before raising the left climber.
		m_leftServo.set(leftSpeed > 0 ? ClimberConstants.kLeftServoUnlockedPosition
				: ClimberConstants.kLeftServoLockedPosition);
		m_rightServo.set(rightSpeed > 0 ? ClimberConstants.kRightServoUnlockedPosition
				: ClimberConstants.kRightServoLockedPosition);

		m_leftClimber.set(leftSpeed);
		m_rightClimber.set(rightSpeed);
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
