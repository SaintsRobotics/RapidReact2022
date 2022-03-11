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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.DutyCycleAbsoluteEncoder;
import frc.robot.MUX;
import frc.robot.MUX.Port;
import frc.robot.REVColorSensorV3;
import frc.robot.Utils;

/** Subsystem that controls the arm, intake, feeders, and shooter flywheel. */
public class ShooterSubsystem extends SubsystemBase {
	private final CANSparkMax m_arm = new CANSparkMax(ShooterConstants.kArmPort, MotorType.kBrushless);
	private final DutyCycleAbsoluteEncoder m_armEncoder = new DutyCycleAbsoluteEncoder(9);

	private final CANSparkMax m_intake = new CANSparkMax(ShooterConstants.kIntakeWheelsPort, MotorType.kBrushless);
	private final MotorControllerGroup m_sideFeeders;
	private final CANSparkMax m_topFeeder = new CANSparkMax(ShooterConstants.kTopFeederPort, MotorType.kBrushless);

	private final WPI_TalonFX m_blackFlywheel = new WPI_TalonFX(ShooterConstants.kBlackFlywheelPort);
	private final WPI_TalonFX m_greenFlywheel = new WPI_TalonFX(ShooterConstants.kGreenFlywheelPort);

	private final MUX m_MUX = new MUX();
	private final REVColorSensorV3 m_proximitySensor = new REVColorSensorV3(m_MUX, Port.kTwo);

	// TODO tune
	private final PIDController m_armPID = new PIDController(ShooterConstants.kArmP, 0, 0);
	private final PIDController m_blackShooterPID = new PIDController(ShooterConstants.kShooterP, 0, 0);
	private final PIDController m_greenShooterPID = new PIDController(ShooterConstants.kShooterP, 0, 0);
	private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.65, 0);

	private boolean m_runningIntake = false;
	private boolean m_reversingIntake = false;
	private Timer m_shootingTimer = new Timer();

	/** Creates a new {@link ShooterSubsystem}. */
	public ShooterSubsystem() {
		m_arm.setIdleMode(IdleMode.kBrake);
		m_arm.setInverted(true);
		m_blackFlywheel.setNeutralMode(NeutralMode.Coast);
		m_greenFlywheel.setNeutralMode(NeutralMode.Coast);
		CANSparkMax leftFeeder = new CANSparkMax(ShooterConstants.kLeftFeederPort, MotorType.kBrushless);
		CANSparkMax rightFeeder = new CANSparkMax(ShooterConstants.kRightFeederPort, MotorType.kBrushless);
		m_intake.setInverted(true);
		leftFeeder.setInverted(true);
		rightFeeder.setInverted(false);
		m_sideFeeders = new MotorControllerGroup(leftFeeder, rightFeeder);
		m_blackShooterPID.setTolerance(50);
		m_greenShooterPID.setTolerance(50);
		m_armPID.setTolerance(2);

		m_shootingTimer.start();
	}

	// top feeder run for how long?
	@Override
	public void periodic() {
		double blackPIDOutput = m_blackShooterPID.calculate(Utils.toRPM(m_blackFlywheel.getSelectedSensorVelocity()));
		double greenPIDOutput = m_blackShooterPID.calculate(Utils.toRPM(m_greenFlywheel.getSelectedSensorVelocity()));
		if (m_blackShooterPID.getSetpoint() > 0) {
			m_blackFlywheel.set(blackPIDOutput + m_feedforward.calculate(m_blackShooterPID.getSetpoint()));
		} else {
			m_blackFlywheel.set(0);
		}

		if ((isShooterPrimed() || m_shootingTimer.get() < 2) && m_blackShooterPID.getSetpoint() > 0
				&& Utils.toRPM(m_blackFlywheel.getSelectedSensorVelocity()) > 0.95
						* ShooterConstants.kBlackShooterSpeedRPM) {
			m_topFeeder.set(ShooterConstants.kTopFeederSpeedFast);
			if (isShooterPrimed()) {
				m_shootingTimer.reset();
			}
			if (m_blackShooterPID.getSetpoint() > 0) {
				m_greenFlywheel.set(greenPIDOutput + m_feedforward.calculate(m_blackShooterPID.getSetpoint()));
			} else {
				m_greenFlywheel.set(0);
			}

			if ((isShooterPrimed() || m_shootingTimer.get() < 2) && m_blackShooterPID.getSetpoint() > 0
					&& Utils.toRPM(m_greenFlywheel.getSelectedSensorVelocity()) > 0.95
							* ShooterConstants.kGreenShooterSpeedRPM) {
				m_topFeeder.set(ShooterConstants.kTopFeederSpeedFast);
				if (isShooterPrimed()) {
					m_shootingTimer.reset();
				}
			}
		} else if (!isShooterPrimed() && m_runningIntake) {
			m_topFeeder.set(ShooterConstants.kTopFeederSpeedSlow);
		} else if (!m_reversingIntake) {
			m_topFeeder.set(0);
		}

		if (OIConstants.kTelemetry) {
			SmartDashboard.putNumber("Black Shooter PID Output", blackPIDOutput);
			SmartDashboard.putNumber("Green Shooter PID Output", greenPIDOutput);

			SmartDashboard.putNumber("Black Shooter Feedforward Output",
					m_feedforward.calculate(m_blackShooterPID.getSetpoint()));
			SmartDashboard.putNumber("Green Shooter Feedforward Output",
					m_feedforward.calculate(m_greenShooterPID.getSetpoint()));

			SmartDashboard.putNumber("Black Shooter Power", m_blackFlywheel.get());
			SmartDashboard.putNumber("Green Shooter Power", m_greenFlywheel.get());

			SmartDashboard.putNumber("Black Shooter RPM", Utils.toRPM(m_blackFlywheel.getSelectedSensorVelocity()));
			SmartDashboard.putNumber("Green Shooter RPM", Utils.toRPM(m_greenFlywheel.getSelectedSensorVelocity()));

			SmartDashboard.putNumber("Black Shooter RPM Error", m_blackShooterPID.getPositionError());
			SmartDashboard.putNumber("Green Shooter RPM Error", m_greenShooterPID.getPositionError());

			SmartDashboard.putNumber("Side Feeder Speed", m_sideFeeders.get());
			SmartDashboard.putNumber("Top Feeder Speed", m_topFeeder.get());
			SmartDashboard.putNumber("Intake Wheel Speed", m_intake.get());
			SmartDashboard.putNumber("Arm Motor Speed", m_arm.get());
			SmartDashboard.putNumber("Arm Encoder", m_armEncoder.getAbsolutePosition());
			SmartDashboard.putNumber("proximity", m_proximitySensor.getProximity());
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
	 * @param speed Speed of the black flywheel in ticks per decisecond.
	 */
	public void setBlackShooterSpeed(double RPM) {
		m_blackShooterPID.setSetpoint(RPM);
		if (RPM == 0) {
			m_sideFeeders.set(0);
		} else {
			m_sideFeeders.set(ShooterConstants.kSideFeederSpeed);
		}
		if (OIConstants.kTelemetry) {
			SmartDashboard.putNumber("Target Shooter Speed", RPM);
		}
	}
	/**
	 * Shoots the ball(s)
	 * 
	 * @param speed Speed of the green flywheel in ticks per decisecond.
	 */
	public void setGreenShooterSpeed(double RPM) {
		m_greenShooterPID.setSetpoint(RPM);
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
		return m_proximitySensor.getProximity() >= 180;
	}

	/*
	 * private boolean isCorrectColor() {
	 * if (DriverStation.getAlliance() == Alliance.Red && m_colorSensor.getRed() >
	 * 300)
	 * return true;
	 * if (DriverStation.getAlliance() == Alliance.Blue && m_colorSensor.getBlue() >
	 * 300)
	 * return true;
	 * 
	 * return false;
	 * }
	 * 
	 * private void printColor() {
	 * if (m_colorSensor.getRed() > 300)
	 * SmartDashboard.putString("color sensed", "red");
	 * if (m_colorSensor.getBlue() > 300)
	 * SmartDashboard.putString("color sensed", "blue");
	 * 
	 * SmartDashboard.putString("color sensed", "none");
	 * }
	 */
}
