// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final class SwerveConstants {
		public static final int kFrontLeftDriveMotorPort = 16;
		public static final int kRearLeftDriveMotorPort = 15;
		public static final int kFrontRightDriveMotorPort = 13;
		public static final int kRearRightDriveMotorPort = 10;

		public static final int kFrontLeftTurningMotorPort = 14;
		public static final int kRearLeftTurningMotorPort = 11;
		public static final int kFrontRightTurningMotorPort = 12;
		public static final int kRearRightTurningMotorPort = 3;

		public static final int kFrontLeftTurningEncoderPort = 17;
		public static final int kRearLeftTurningEncoderPort = 19;
		public static final int kFrontRightTurningEncoderPort = 18;
		public static final int kRearRightTurningEncoderPort = 20;

		public static final boolean kFrontLeftDriveMotorReversed = false;
		public static final boolean kRearLeftDriveMotorReversed = false;
		public static final boolean kFrontRightDriveMotorReversed = true;
		public static final boolean kRearRightDriveMotorReversed = true;

		public static final double kFrontLeftTurningEncoderOffset = -327;
		public static final double kRearLeftTurningEncoderOffset = -261;
		public static final double kFrontRightTurningEncoderOffset = -253;
		public static final double kRearRightTurningEncoderOffset = -207;

		/** Distance between centers of right and left wheels on robot. */
		public static final double kTrackWidth = 0.57;

		/** Distance between front and back wheels on robot. */
		public static final double kWheelBase = 0.6;

		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

		public static final double kMaxSpeedMetersPerSecond = 3.66;

		// TODO update value with new robot
		public static final double kMaxAngularSpeedRadiansPerSecond = 8.76;

		/**
		 * Time in seconds for the robot to stop turning from max speed.
		 * TODO update value with new robot.
		 */
		public static final double kTurningStopTime = 0.2;
	}

	public static final class ModuleConstants {
		public static final double kWheelDiameterMeters = 0.1;

		/** Gear ratio between the motor and the wheel. */
		public static final double kDrivingGearRatio = 8.14;
	}

	public static final class ShooterConstants {
		public static final int kShooterMotorPort = 9;

		public static final double kShooterSpeedTicksPerDecisecond = 12000;
	}

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
		public static final double kControllerDeadband = 0.11;
	}
}
