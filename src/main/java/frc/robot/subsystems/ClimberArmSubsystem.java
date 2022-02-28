// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberArmSubsystem extends SubsystemBase {
	private CANSparkMax m_leftClimberArm = new CANSparkMax(ClimberConstants.kLeftArmPort, MotorType.kBrushless);
	private CANSparkMax m_rightClimberArm = new CANSparkMax(ClimberConstants.kRightArmPort, MotorType.kBrushless);

	private CANCoder m_leftEncoder = new CANCoder(456);
	private CANCoder m_rightEncoder = new CANCoder(123);

	private PIDController m_leftPID = new PIDController(0.3, 0, 0);
	private PIDController m_rightPID = new PIDController(0.3, 0, 0);
	
	private Servo m_leftServo = new Servo(345);
	private Servo m_rightServo = new Servo(789);

	/** Creates a new {@link ClimberArmSubsystem}. */
	public ClimberArmSubsystem() {
	}

	/**
	 * Sets the speed of the climber.
	 * 
	 * @param speed Speed of the climber from -1 to 1.
	 */
	public void setSpeed(double speed) {
		m_leftClimberArm.set(speed);
		m_rightClimberArm.set(speed);
		SmartDashboard.putNumber("Desired Climber Speed", speed);
	}

	public void setPosition(double position) {
		m_leftPID.setSetpoint(position);
		m_rightPID.setSetpoint(position);
		m_leftClimberArm.set(m_leftPID.calculate(m_leftEncoder.getPosition()));
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Climber Speed", m_climberMotor.get());
	}
}
