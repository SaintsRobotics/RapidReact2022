// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
	private final CANSparkMax m_armMotor = new CANSparkMax(IntakeConstants.kArmPort, MotorType.kBrushless);
	private final CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.kIntakeWheelsPort, MotorType.kBrushless);
	private final CANSparkMax m_topFeederMotor = new CANSparkMax(IntakeConstants.kTopFeederPort, MotorType.kBrushless);
	private final MotorControllerGroup m_sideFeeders;

	// TODO tune PID
	private final PIDController m_armPID = new PIDController(0.3, 0, 0);

	/** Creates a new {@link IntakeSubsystem}. */
	public IntakeSubsystem() {
		m_armMotor.setIdleMode(IdleMode.kBrake);
		m_armPID.setTolerance(0.05);
		CANSparkMax leftFeeder = new CANSparkMax(IntakeConstants.kLeftFeederPort, MotorType.kBrushless);
		CANSparkMax rightFeeder = new CANSparkMax(IntakeConstants.kRightFeederPort, MotorType.kBrushless);
		m_sideFeeders = new MotorControllerGroup(leftFeeder, rightFeeder);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Intake Wheel Speed", m_intakeMotor.get());
		SmartDashboard.putNumber("Arm Motor Speed", m_armMotor.get());
	}

	/** Raises the arm. */
	public void raiseArm() {
		// m_armPID.setSetpoint(IntakeConstants.kRaisedArmAngle);
		m_armMotor.set(0.5);
	}

	/** Lowers the arm. */
	public void lowerArm() {
		// m_armPID.setSetpoint(IntakeConstants.kLoweredArmAngle);
		m_armMotor.set(-0.3);
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

	/** Runs the side feeders. */
	public void sideFeed() {
		m_sideFeeders.set(IntakeConstants.kFeederSpeed);
	}

	/** Turns off the side feeders. */
	public void sideFeedOff() {
		m_sideFeeders.set(0);
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
