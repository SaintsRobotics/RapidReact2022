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
	private final PIDController m_bottomShooterPID = new PIDController(ShooterConstants.kBottomShooterPTarmac, 0, 0);
	private final PIDController m_topShooterPID = new PIDController(ShooterConstants.kTopShooterPTarmac, 0, 0);
	private SimpleMotorFeedforward m_bottomFeedforward = new SimpleMotorFeedforward(
			ShooterConstants.kBottomFeedforwardTarmac, 0);
	private SimpleMotorFeedforward m_topFeedforward = new SimpleMotorFeedforward(ShooterConstants.kTopFeedforwardTarmac,
			0);

	private boolean m_runningIntake = false;
	private boolean m_reversingIntake = false;
	private Timer m_feederTimer = new Timer();

	public enum ShootingMode {
		kFender,
		kTarmac,
		kStop;
	}

	/** Creates a new {@link ShooterSubsystem}. */
	public ShooterSubsystem() {
		// TODO change to getAngle if WPILib adds it
		m_armEncoder.setDistancePerRotation(360);

		m_arm.setIdleMode(IdleMode.kBrake);
		m_bottomFlywheel.setNeutralMode(NeutralMode.Coast);
		m_topFlywheel.setNeutralMode(NeutralMode.Coast);

		m_arm.setInverted(ShooterConstants.kArmReversed);
		m_intake.setInverted(ShooterConstants.kIntakeReversed);
		m_leftFeeder.setInverted(ShooterConstants.kLeftFeederReversed);
		m_rightFeeder.setInverted(ShooterConstants.kRightFeederReversed);
		m_topFeeder.setInverted(ShooterConstants.kTopFeederReversed);
		m_bottomFlywheel.setInverted(ShooterConstants.kBottomFlywheelReversed);
		m_topFlywheel.setInverted(ShooterConstants.kTopFlywheelReversed);

		m_bottomShooterPID.setTolerance(168, 5000);
		m_topShooterPID.setTolerance(174.4, 5000);
		m_armPID.setTolerance(2);
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

		if (m_feederTimer.get() > 0) {
			if (m_bottomShooterPID.atSetpoint() && m_topShooterPID.atSetpoint()) {
				m_topFeeder.set(ShooterConstants.kTopFeederSpeedFast);
			}

			if (m_feederTimer.get() > 3) {
				m_feederTimer.stop();
				m_feederTimer.reset();
			}
		} else if (m_bottomShooterPID.atSetpoint() && m_topShooterPID.atSetpoint() && isShooterPrimed()
				&& m_bottomShooterPID.getSetpoint() > 0) {
			m_feederTimer.start();
		} else if (!isShooterPrimed() && m_runningIntake) { // if we're trying to intake - prime the first ball
			m_topFeeder.set(ShooterConstants.kTopFeederSpeedSlow);
		} else if (!m_reversingIntake) { // as long as we're not trying to spit out the wrong color, set to zero
			m_topFeeder.set(0);
		}

		if (Robot.isReal()) {
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
	 * Sets the {@link ShootingMode mode} for the shooter.
	 * 
	 * @param mode {@link ShootingMode Mode} for the shooter.
	 */
	public void setShootingMode(ShootingMode mode) {
		switch (mode) {
			case kFender:
				m_bottomShooterPID.setP(ShooterConstants.kBottomShooterPFender);
				m_topShooterPID.setP(ShooterConstants.kTopShooterPFender);

				m_bottomShooterPID.setSetpoint(ShooterConstants.kBottomMotorRPMFender);
				m_topShooterPID.setSetpoint(ShooterConstants.kTopMotorRPMFender);

				m_bottomFeedforward = new SimpleMotorFeedforward(ShooterConstants.kBottomFeedforwardFender, 0);
				m_topFeedforward = new SimpleMotorFeedforward(ShooterConstants.kTopFeedforwardFender, 0);
				break;
			case kTarmac:
				m_bottomShooterPID.setP(ShooterConstants.kBottomShooterPTarmac);
				m_topShooterPID.setP(ShooterConstants.kTopShooterPTarmac);

				m_bottomShooterPID.setSetpoint(ShooterConstants.kBottomMotorRPMTarmac);
				m_topShooterPID.setSetpoint(ShooterConstants.kTopMotorRPMTarmac);

				m_bottomFeedforward = new SimpleMotorFeedforward(ShooterConstants.kBottomFeedforwardTarmac, 0);
				m_topFeedforward = new SimpleMotorFeedforward(ShooterConstants.kTopFeedforwardTarmac, 0);
				break;
			case kStop:
				m_bottomShooterPID.setSetpoint(0);
				m_topShooterPID.setSetpoint(0);
				break;
		}

		if (m_bottomShooterPID.getSetpoint() == 0) {
			m_sideFeeders.set(0);
		} else {
			m_sideFeeders.set(ShooterConstants.kSideFeederSpeed);
		}
	}

	private boolean isShooterPrimed() {
		return m_shooterColorSensor.getProximity() >= 142;
	}

	public void topFeederOn() {
		m_topFeeder.set(ShooterConstants.kTopFeederSpeedFast);
	}

	public void topFeederOff() {
		m_topFeeder.set(0);
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
