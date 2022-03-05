// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberArmSubsystem extends SubsystemBase {
	private CANSparkMax m_leftClimberArm = new CANSparkMax(ClimberConstants.kLeftArmPort, MotorType.kBrushless);
	private CANSparkMax m_rightClimberArm = new CANSparkMax(ClimberConstants.kRightArmPort, MotorType.kBrushless);

	private CANCoder m_leftEncoder = new CANCoder(ClimberConstants.kLeftEncoderPort);
	private CANCoder m_rightEncoder = new CANCoder(ClimberConstants.kRightEncoderPort);

	private PIDController m_leftPID = new PIDController(0.3, 0, 0);
	private PIDController m_rightPID = new PIDController(0.3, 0, 0);
	
	private Servo m_leftServo = new Servo(ClimberConstants.kLeftServoPort);
	private Servo m_rightServo = new Servo(ClimberConstants.kRightServoPort);

	private double m_leftSpeed;
	private double m_rightSpeed;

	public boolean should_lock;

	/** Creates a new {@link ClimberArmSubsystem}. */
	public ClimberArmSubsystem() {
		should_lock = true;
	}

	public void realignArms() {
		if (m_leftEncoder.getPosition() < m_rightEncoder.getPosition()) {
			m_leftPID.setSetpoint(m_rightEncoder.getPosition());
			m_leftSpeed = m_leftPID.calculate(m_leftEncoder.getPosition());
		} else {
			m_rightPID.setSetpoint(m_leftEncoder.getPosition());
			m_rightSpeed = m_rightPID.calculate(m_rightEncoder.getPosition());
		}
	}

	public void setSpeed(double speed){
		m_leftSpeed = speed;
		m_rightSpeed = speed;

	}

	public void releaseServos(){
		m_leftServo.set(ClimberConstants.kServoReleasedPos);
		m_rightServo.set(ClimberConstants.kServoReleasedPos);
		should_lock = false;		
	}

	public void lockServos(){
		m_leftServo.set(ClimberConstants.kServoLockedPos);
		m_rightServo.set(ClimberConstants.kServoLockedPos);
		should_lock = true;
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Climber Speed", m_leftClimberArm.get()); //Same as right arm
		if (should_lock) {
		 	m_leftClimberArm.set(0);
		 	m_rightClimberArm.set(0);	
		} else {
		 	m_leftClimberArm.set(m_leftSpeed);
		 	m_rightClimberArm.set(m_rightSpeed);
		}
		m_leftClimberArm.set(0);
		m_rightClimberArm.set(0);
		SmartDashboard.putBoolean("should_lock", should_lock);
		SmartDashboard.putNumber("Desired Climber Speed", m_leftSpeed);
		SmartDashboard.putNumber("Left Servo Position", m_leftServo.get());
		SmartDashboard.putNumber("Right Servo Position", m_rightServo.get());
		SmartDashboard.putNumber("Left Encoder", m_leftEncoder.getPosition());
		SmartDashboard.putNumber("Right Encoder", m_rightEncoder.getPosition());
	}
}
