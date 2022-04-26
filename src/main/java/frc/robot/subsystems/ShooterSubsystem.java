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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.MUX;
import frc.robot.REVColorSensorV3;
import frc.robot.Robot;

/** Subsystem that controls the arm, intake, feeders, and shooter flywheel. */
public class ShooterSubsystem extends SubsystemBase {
	private final CANSparkMax m_arm = new CANSparkMax(ShooterConstants.kArmPort, MotorType.kBrushless);
	private final DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(9);

	private final CANSparkMax m_intake = new CANSparkMax(ShooterConstants.kIntakePort, MotorType.kBrushless);

	private final CANSparkMax m_leftFeeder = new CANSparkMax(ShooterConstants.kLeftFeederPort, MotorType.kBrushless);
	private final CANSparkMax m_rightFeeder = new CANSparkMax(ShooterConstants.kRightFeederPort, MotorType.kBrushless);
	private final MotorControllerGroup m_sideFeeders = new MotorControllerGroup(m_leftFeeder, m_rightFeeder);

	private final CANSparkMax m_topFeeder = new CANSparkMax(ShooterConstants.kTopFeederPort, MotorType.kBrushless);

	private final WPI_TalonFX m_bottomFlywheel = new WPI_TalonFX(ShooterConstants.kBottomFlywheelPort);
	private final WPI_TalonFX m_topFlywheel = new WPI_TalonFX(ShooterConstants.kTopFlywheelPort);

	private final MUX m_MUX = new MUX();
	private final REVColorSensorV3 m_queueColorSensor = new REVColorSensorV3(m_MUX,
			ShooterConstants.kQueueColorSensorPort);
	private final REVColorSensorV3 m_shooterColorSensor = new REVColorSensorV3(m_MUX,
			ShooterConstants.kShooterColorSensorPort);

	private final PIDController m_armPID = new PIDController(ShooterConstants.kArmP, 0, 0);
	private final PIDController m_bottomShooterPID = new PIDController(ShooterConstants.kBottomShooterP, 0, 0);
	private final PIDController m_topShooterPID = new PIDController(ShooterConstants.kTopShooterP, 0, 0);
	private final SimpleMotorFeedforward m_bottomFeedforward = new SimpleMotorFeedforward(
			ShooterConstants.kBottomFlywheelFeedforwardS, 0);
	private final SimpleMotorFeedforward m_topFeedforward = new SimpleMotorFeedforward(
			ShooterConstants.kTopFlywheelFeedforwardS, 0);

	private final Timer m_shooterColorTimer = new Timer();
	private final Timer m_intakeTimer = new Timer();
	private final Timer m_feederTimer = new Timer();

	/** Creates a new {@link ShooterSubsystem}. */
	public ShooterSubsystem() {
		// TODO change to getAngle if WPILib adds it
		m_armEncoder.setDistancePerRotation(360);

		m_arm.setIdleMode(IdleMode.kBrake);
		// TODO see if set to coast by default
		m_bottomFlywheel.setNeutralMode(NeutralMode.Coast);
		m_topFlywheel.setNeutralMode(NeutralMode.Coast);

		m_arm.setInverted(ShooterConstants.kArmReversed);
		m_intake.setInverted(ShooterConstants.kIntakeReversed);
		m_leftFeeder.setInverted(ShooterConstants.kLeftFeederReversed);
		m_rightFeeder.setInverted(ShooterConstants.kRightFeederReversed);
		m_topFeeder.setInverted(ShooterConstants.kTopFeederReversed);
		m_bottomFlywheel.setInverted(ShooterConstants.kBottomFlywheelReversed);
		m_topFlywheel.setInverted(ShooterConstants.kTopFlywheelReversed);

		m_armPID.setTolerance(ShooterConstants.kArmToleranceDegrees);
		m_bottomShooterPID.setTolerance(ShooterConstants.kFlywheelToleranceRPM);
		m_topShooterPID.setTolerance(ShooterConstants.kFlywheelToleranceRPM);

		m_armPID.enableContinuousInput(-180, 180);
	}

	@Override
	public void periodic() {
		// TODO tune PID so we don't need to call MathUtil.clamp
		// Stops the arm if it is at the setpoint to avoid draining power.
		if (m_armPID.atSetpoint()) {
			m_arm.set(0);
		} else if (m_armPID.getSetpoint() == ShooterConstants.kUpperArmAngle) {
			m_arm.set(MathUtil.clamp(m_armPID.calculate(m_armEncoder.getDistance()), -0.5, -0.1));
		} else if (m_armPID.getSetpoint() == ShooterConstants.kLowerArmAngle) {
			m_arm.set(MathUtil.clamp(m_armPID.calculate(m_armEncoder.getDistance()), 0.1, 0.7));
		}

		// Disables intake if arm is raised enough.
		if (m_armEncoder.getDistance() > ShooterConstants.kArmIntakeAngle) {
			m_intake.set(0);
		}

		final boolean queueIsBlue = m_queueColorSensor.getBlue() > ShooterConstants.kBlueThreshold;
		final boolean queueIsRed = m_queueColorSensor.getRed() > ShooterConstants.kRedThreshold;
		final boolean shooterIsBlue = m_shooterColorSensor.getBlue() > ShooterConstants.kBlueThreshold;
		final boolean shooterIsRed = m_shooterColorSensor.getRed() > ShooterConstants.kRedThreshold;

		// Spits out incorrect color of ball.
		if ((shooterIsBlue && DriverStation.getAlliance() == Alliance.Red) ||
				(shooterIsRed && DriverStation.getAlliance() == Alliance.Blue)) {
			m_shooterColorTimer.reset();
			m_shooterColorTimer.start();
			m_intake.set(-ShooterConstants.kIntakeSpeed);
			m_topFeeder.set(-ShooterConstants.kTopFeederSpeedFast);
			m_sideFeeders.set(-ShooterConstants.kSideFeederSpeed);
		} else if ((queueIsBlue && DriverStation.getAlliance() == Alliance.Red) ||
				(queueIsRed && DriverStation.getAlliance() == Alliance.Blue)) {
			m_intake.set(-ShooterConstants.kIntakeSpeed);
			m_sideFeeders.set(-ShooterConstants.kSideFeederSpeed);
		}
		// Runs both feeders when shooter is at the setpoint.
		else if (m_bottomShooterPID.atSetpoint() &&
				m_topShooterPID.atSetpoint() &&
				m_bottomShooterPID.getSetpoint() != 0 &&
				m_topShooterPID.getSetpoint() != 0) {
			m_topFeeder.set(ShooterConstants.kTopFeederSpeedFast);
			m_sideFeeders.set(ShooterConstants.kSideFeederSpeed);
		}
		// If intaking.
		else if (m_intakeTimer.get() < 1) {
			if (m_shooterColorSensor.getProximity() < ShooterConstants.kShooterColorSensorProximity) {
				m_topFeeder.set(ShooterConstants.kTopFeederSpeedFast);
			}
			m_sideFeeders.set(ShooterConstants.kSideFeederSpeed);
		} else {
			m_topFeeder.set(0);
			m_sideFeeders.set(0);
		}

		// Disables flywheels if the setpoint is 0.
		if (m_bottomShooterPID.getSetpoint() > 0) {
			m_bottomFlywheel.set(
					m_bottomShooterPID.calculate(toRPM(m_bottomFlywheel.getSelectedSensorVelocity()))
							+ m_bottomFeedforward.calculate(m_bottomShooterPID.getSetpoint()));
			m_topFlywheel.set(
					m_topShooterPID.calculate(toRPM(m_topFlywheel.getSelectedSensorVelocity()))
							+ m_topFeedforward.calculate(m_topShooterPID.getSetpoint()));
		} else {
			m_bottomFlywheel.set(0);
			m_topFlywheel.set(0);
		}

		if (Robot.isReal()) {
			if (DriverStation.isTest()) {
				SmartDashboard.putNumber("Arm Angle", m_armEncoder.getDistance());
				SmartDashboard.putNumber("Queue Proximity", m_queueColorSensor.getProximity());
				SmartDashboard.putNumber("Shooter Proximity", m_shooterColorSensor.getProximity());
			}

			SmartDashboard.putNumber("Temperature Arm", m_arm.getMotorTemperature());
			SmartDashboard.putNumber("Temperature Intake", m_intake.getMotorTemperature());
			SmartDashboard.putNumber("Temperature Left Feeder", m_leftFeeder.getMotorTemperature());
			SmartDashboard.putNumber("Temperature Right Feeder", m_rightFeeder.getMotorTemperature());
			SmartDashboard.putNumber("Temperature Top Feeder", m_topFeeder.getMotorTemperature());
			SmartDashboard.putNumber("Temperature Bottom Flywheel", m_topFlywheel.getTemperature());
			SmartDashboard.putNumber("Temperature Top Flywheel", m_bottomFlywheel.getTemperature());

			SmartDashboard.putNumber("Current Arm", m_arm.getOutputCurrent());
			SmartDashboard.putNumber("Current Intake", m_intake.getOutputCurrent());
			SmartDashboard.putNumber("Current Left Feeder", m_leftFeeder.getOutputCurrent());
			SmartDashboard.putNumber("Current Right Feeder", m_rightFeeder.getOutputCurrent());
			SmartDashboard.putNumber("Current Top Feeder", m_topFeeder.getOutputCurrent());
			SmartDashboard.putNumber("Current Bottom Flywheel", m_topFlywheel.getStatorCurrent());
			SmartDashboard.putNumber("Current Top Flywheel", m_bottomFlywheel.getStatorCurrent());

			SmartDashboard.putBoolean("queue is blue", queueIsBlue);
			SmartDashboard.putBoolean("queue is red", queueIsRed);
			SmartDashboard.putBoolean("shooter is blue", shooterIsBlue);
			SmartDashboard.putBoolean("shooter is red", shooterIsRed);
		}
	}

	/** Raises the arm. */
	public void raiseArm() {
		m_armPID.setSetpoint(ShooterConstants.kUpperArmAngle);
	}

	/** Lowers the arm. */
	public void lowerArm() {
		m_armPID.setSetpoint(ShooterConstants.kLowerArmAngle);
	}

	/** Runs the intake. */
	public void intake() {
		m_feederTimer.reset();
		m_feederTimer.start();
		m_intakeTimer.reset();
		m_intakeTimer.start();
		m_intake.set(ShooterConstants.kIntakeSpeed);
		m_sideFeeders.set(ShooterConstants.kSideFeederSpeed);
	}

	/** Runs the intake in reverse. */
	public void intakeReverse() {
		m_intake.set(-ShooterConstants.kIntakeSpeed);
	}

	/** Turns off the intake. */
	public void intakeOff() {
		m_intake.set(0);
	}

	/**
	 * Sets the RPM of the flywheels.
	 * 
	 * @param bottomRPM RPM for the bottom flywheel.
	 * @param topRPM    RPM for the top flywheel.
	 */
	public void setFlywheelRPM(double bottomRPM, double topRPM) {
		m_bottomShooterPID.setSetpoint(bottomRPM);
		m_topShooterPID.setSetpoint(topRPM);
	}

	/**
	 * Converts the speed of a TalonFX from the default units of ticks per
	 * decisecond to RPM.
	 * 
	 * @param ticksPerDecisecond The speed in ticks per decisecond.
	 * @return The speed in RPM.
	 */
	private double toRPM(double ticksPerDecisecond) {
		return ticksPerDecisecond * 600 / 2048;
	}
}
