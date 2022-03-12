// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.DutyCycleAbsoluteEncoder;
import frc.robot.MUX;
import frc.robot.REVColorSensorV3;
import frc.robot.Utils;

/** Subsystem that controls the arm, intake, feeders, and shooter flywheel. */
public class ShooterSubsystem extends SubsystemBase {
	private final CANSparkMax m_arm = new CANSparkMax(ShooterConstants.kArmPort, MotorType.kBrushless);
	private final DutyCycleAbsoluteEncoder m_armEncoder = new DutyCycleAbsoluteEncoder(9);

	private final CANSparkMax m_intake = new CANSparkMax(ShooterConstants.kIntakeWheelsPort, MotorType.kBrushless);
	private final MotorControllerGroup m_sideFeeders;
	private final CANSparkMax m_topFeeder = new CANSparkMax(ShooterConstants.kTopFeederPort, MotorType.kBrushless);

	private final WPI_TalonFX m_flywheel = new WPI_TalonFX(ShooterConstants.kFlywheelPort);

	private final MUX m_MUX = new MUX();
	private final REVColorSensorV3 m_queueColorSensor = new REVColorSensorV3(m_MUX,
			ShooterConstants.kQueueColorSensorPort);
	private final REVColorSensorV3 m_shooterColorSensor = new REVColorSensorV3(m_MUX,
			ShooterConstants.kShooterColorSensorPort);

	// TODO tune
	private final PIDController m_armPID = new PIDController(0.005, 0, 0);
	private final PIDController m_shooterPID = new PIDController(0.0005, 0, 0);
	private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.65, 0);

	private boolean m_runningIntake = false;
	private boolean m_reversingIntake = false;
	private Timer m_shootingTimer = new Timer();

	/** Creates a new {@link ShooterSubsystem}. */
	public ShooterSubsystem() {
		m_arm.setIdleMode(IdleMode.kBrake);
		m_arm.setInverted(true);
		m_flywheel.setNeutralMode(NeutralMode.Coast);
		CANSparkMax leftFeeder = new CANSparkMax(ShooterConstants.kLeftFeederPort, MotorType.kBrushless);
		CANSparkMax rightFeeder = new CANSparkMax(ShooterConstants.kRightFeederPort, MotorType.kBrushless);
		m_intake.setInverted(true);
		leftFeeder.setInverted(true);
		rightFeeder.setInverted(false);
		m_sideFeeders = new MotorControllerGroup(leftFeeder, rightFeeder);
		m_shooterPID.setTolerance(50);
		m_armPID.setTolerance(2);

		m_shootingTimer.start();
	}

	// top feeder run for how long?
	@Override
	public void periodic() {
		final boolean queueIsBlue = m_queueColorSensor.getBlue() > ShooterConstants.kBlueThreshold;
		final boolean queueIsRed = m_queueColorSensor.getRed() > ShooterConstants.kRedThreshold;
		final boolean shooterIsBlue = m_shooterColorSensor.getBlue() > ShooterConstants.kBlueThreshold;
		final boolean shooterIsRed = m_shooterColorSensor.getRed() > ShooterConstants.kRedThreshold;

		// Checks if the color of ball is opposite that of the alliance.
		if ((queueIsBlue && DriverStation.getAlliance() == Alliance.Red) ||
				(queueIsRed && DriverStation.getAlliance() == Alliance.Blue) ||
				(shooterIsBlue && DriverStation.getAlliance() == Alliance.Red) ||
				(shooterIsRed && DriverStation.getAlliance() == Alliance.Blue)) {
			// TODO also run the feeders in reverse
			intakeReverse();
		}

		double pidOutput = m_shooterPID.calculate(Utils.toRPM(m_flywheel.getSelectedSensorVelocity()));
		if (m_shooterPID.getSetpoint() > 0) {
			m_flywheel.set(pidOutput + m_feedforward.calculate(m_shooterPID.getSetpoint()));
		} else {
			m_flywheel.set(0);
		}

		if ((isShooterPrimed() || m_shootingTimer.get() < 2) && m_shooterPID.getSetpoint() > 0
				&& Utils.toRPM(m_flywheel.getSelectedSensorVelocity()) > 0.95 * ShooterConstants.kShooterSpeedRPM) {
			m_topFeeder.set(ShooterConstants.kTopFeederSpeedFast);
			if (isShooterPrimed()) {
				m_shootingTimer.reset();
			}
		} else if (!isShooterPrimed() && m_runningIntake) {
			m_topFeeder.set(ShooterConstants.kTopFeederSpeedSlow);
		} else if (!m_reversingIntake) {
			m_topFeeder.set(0);
		}

		if (OIConstants.kTelemetry) {
			SmartDashboard.putNumber("Shooter PID Output", pidOutput);
			SmartDashboard.putNumber("Shooter PID velocity error", m_shooterPID.getVelocityError());
			SmartDashboard.putNumber("Shooter PID position error", m_shooterPID.getPositionError());
			SmartDashboard.putNumber("Shooter Feedforward output", m_feedforward.calculate(m_shooterPID.getSetpoint()));
			SmartDashboard.putNumber("Shooter Power", m_flywheel.get());
			SmartDashboard.putNumber("Shooter RPM", Utils.toRPM(m_flywheel.getSelectedSensorVelocity()));
			SmartDashboard.putNumber("Side Feeder Speed", m_sideFeeders.get());
			SmartDashboard.putNumber("Top Feeder Speed", m_topFeeder.get());
			SmartDashboard.putNumber("Intake Wheel Speed", m_intake.get());
			SmartDashboard.putNumber("Arm Motor Speed", m_arm.get());
			SmartDashboard.putNumber("Arm Encoder", m_armEncoder.getAbsolutePosition());

			SmartDashboard.putNumber("Queue Proximity", m_queueColorSensor.getProximity());
			SmartDashboard.putBoolean("Queue Is Blue", queueIsBlue);
			SmartDashboard.putBoolean("Queue Is Red", queueIsRed);

			SmartDashboard.putNumber("Shooter Proximity", m_shooterColorSensor.getProximity());
			SmartDashboard.putBoolean("Shooter Is Blue", shooterIsBlue);
			SmartDashboard.putBoolean("Shooter Is Red", shooterIsRed);

			SmartDashboard.putBoolean("is shooter primed", isShooterPrimed());
		}
	}

	/** Raises the arm. */
	public void raiseArm() {
		m_armPID.setSetpoint(ShooterConstants.kUpperArmAngle);
		if (m_armPID.atSetpoint())
			m_arm.set(0);
		else
			m_arm.set(MathUtil.clamp(m_armPID.calculate(m_armEncoder.getAbsolutePosition()), -0.25, -0.1));
	}

	/** Lowers the arm. */
	public void lowerArm() {
		m_armPID.setSetpoint(ShooterConstants.kLowerArmAngle);
		if (m_armPID.atSetpoint())
			m_arm.set(0);
		else
			m_arm.set(MathUtil.clamp(m_armPID.calculate(m_armEncoder.getAbsolutePosition()), 0.1, 0.25));
	}

	public void stopArm() {
		m_arm.set(0);
	}

	/** Runs the intake. */
	public void intake() {
		// if there is a ball at the top: don't run top feeder
		// run the intake wheels and side feeders
		m_runningIntake = true;
		m_sideFeeders.set(ShooterConstants.kSideFeederSpeed);
		m_intake.set(ShooterConstants.kIntakeSpeed);
	}

	/** Runs the intake in reverse. */
	public void intakeReverse() {
		m_reversingIntake = true;
		m_intake.set(-ShooterConstants.kIntakeSpeed);
		m_sideFeeders.set(-ShooterConstants.kSideFeederSpeed);
		m_topFeeder.set(-ShooterConstants.kTopFeederSpeedFast);
	}

	/** Turns off the intake. */
	public void intakeOff() {
		m_runningIntake = false;
		m_reversingIntake = false;
		m_intake.set(0);
		m_sideFeeders.set(0);
	}

	/**
	 * Shoots the ball(s)
	 * 
	 * @param speed Speed of the shooter in ticks per decisecond.
	 */
	public void setShooterSpeed(double RPM) {
		m_shooterPID.setSetpoint(RPM);
		if (RPM == 0) {
			m_sideFeeders.set(0);
		} else {
			m_sideFeeders.set(ShooterConstants.kSideFeederSpeed);
		}
		if (OIConstants.kTelemetry) {
			SmartDashboard.putNumber("Target Shooter Speed", RPM);
		}
	}

	private boolean isShooterPrimed() {
		return m_queueColorSensor.getProximity() >= 180;
	}
}
