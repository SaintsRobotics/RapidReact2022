// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/** Subsystem that controls the intake, arm, and feeder of the robot. */
public class IntakeSubsystem extends SubsystemBase {
	private final CANSparkMax m_armMotor = new CANSparkMax(IntakeConstants.kArmPort, MotorType.kBrushless);
	private final CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.kWheelsPort, MotorType.kBrushless);
	private final RelativeEncoder m_armEncoder = m_armMotor.getEncoder();
	private final CANSparkMax m_topFeederMotor = new CANSparkMax(IntakeConstants.kTopFeederPort, MotorType.kBrushless);
	private final MotorControllerGroup m_sideFeeders;

	// TODO tune PID
	private final PIDController m_armPID = new PIDController(0.3, 0, 0);

	/** Creates a new {@link IntakeSubsystem}. */
	public IntakeSubsystem() {
		m_armMotor.setIdleMode(IdleMode.kBrake);
		m_armPID.setTolerance(0.05);
		m_armEncoder.setPositionConversionFactor(2 * Math.PI); // converts "rotations" to radians
		CANSparkMax leftFeeder = new CANSparkMax(IntakeConstants.kLeftFeederPort, MotorType.kBrushless);
		CANSparkMax rightFeeder = new CANSparkMax(IntakeConstants.kRightFeederPort, MotorType.kBrushless);
		m_sideFeeders = new MotorControllerGroup(leftFeeder, rightFeeder);
	}

	@Override
	public void periodic() {
		// TODO use encoder to determine arm position
		m_armMotor.set(m_armPID.calculate(0));
		SmartDashboard.putNumber("top", m_topFeederMotor.get());
	}

	/** Raises the arm and turns off the intake. */
	public void raiseArm() {
		m_armPID.setSetpoint(IntakeConstants.kRaisedArmAngle);
		IntakeOff();
	}

	/** Lowers the arm. */
	public void lowerArm() {
		m_armPID.setSetpoint(IntakeConstants.kLoweredArmAngle);
	}

	/** Runs the intake if the arm is lowered. */
	public void intake() {
		m_intakeMotor.set(m_armPID.atSetpoint() && m_armPID.getSetpoint() == IntakeConstants.kLoweredArmAngle
				? IntakeConstants.kIntakeSpeed
				: 0);
	}

	/** Runs the intake in reverse if the arm is lowered. */
	public void intakeReverse() {
		m_intakeMotor.set(m_armPID.atSetpoint() && m_armPID.getSetpoint() == IntakeConstants.kLoweredArmAngle
				? -IntakeConstants.kIntakeSpeed
				: 0);
	}

	/** Turns off the intake. */
	public void IntakeOff() {
		m_intakeMotor.set(0);
	}

	/** Runs the top feeder. */
	public void topFeed() {
		m_topFeederMotor.set(IntakeConstants.kFeederSpeed);
	}

	/** Turns off the top feeder. */
	public void topFeedOff() {
		m_topFeederMotor.set(0);
	}
}
