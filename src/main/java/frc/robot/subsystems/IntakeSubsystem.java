// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
	private final CANSparkMax m_armMotor = new CANSparkMax(24, MotorType.kBrushless);
	private final CANSparkMax m_intakeMotor = new CANSparkMax(25, MotorType.kBrushless);
	private final RelativeEncoder m_armEncoder = m_armMotor.getEncoder();

	// TODO tune PID
	private final PIDController m_armPID = new PIDController(0.3, 0, 0);

	/** Creates a new {@link IntakeSubsystem}. */
	public IntakeSubsystem() {
		m_armMotor.setIdleMode(IdleMode.kBrake);
		m_armPID.setTolerance(0.05);
		m_armEncoder.setPositionConversionFactor(2 * Math.PI); // converts "rotations" to radians
	}

	@Override
	public void periodic() {
		m_armMotor.set(m_armPID.calculate(m_armEncoder.getPosition()));
		SmartDashboard.putNumber("Intake Wheel Speed", m_intakeMotor.get());
		SmartDashboard.putNumber("Arm Motor Speed", m_armMotor.get());
		SmartDashboard.putNumber("Arm Position", m_armEncoder.getPosition());

	}

	/** Raises the arm. */
	public void raiseArm() {
		m_armPID.setSetpoint(IntakeConstants.kRaisedArmAngle);
	}

	/** Lowers the arm. */
	public void lowerArm() {
		m_armPID.setSetpoint(IntakeConstants.kLoweredArmAngle);
	}

	/** Runs the intake. */
	public void intake() {
		m_intakeMotor.set(IntakeConstants.kIntakeSpeed);
	}

	/** Runs the intake in reverse. */
	public void intakeReverse() {
		m_intakeMotor.set(-IntakeConstants.kIntakeSpeed);
	}

	/** Turns off the intake. */
	public void intakeOff() {
		m_intakeMotor.set(0);
	}

}
