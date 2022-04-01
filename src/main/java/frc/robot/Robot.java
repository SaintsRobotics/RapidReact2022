// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private Thread m_visionThread;
	private RobotContainer m_robotContainer;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_robotContainer = new RobotContainer();
		CameraServer.startAutomaticCapture();

		m_visionThread = new Thread(){
			public void run(){
				//Set up the camera server
				UsbCamera camera = CameraServer.startAutomaticCapture();

				//Set the resolution
				camera.setResolution(640, 480);

				//Set up the Sink for the video
				CvSink cvSink = CameraServer.getVideo();

				//Create the matrix for the image
				Mat mat = new Mat();

				//Create the parallel thread that manages video capturing.
				while (!Thread.interrupted()) {

					//Print out an error message if the frame rate is too low. Restart loop if this is so
					if (cvSink.grabFrame(mat) == 0) {
						System.out.println("Frame not found!");
						continue;
					}

					//Preprocess the image
					Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2GRAY);
					Imgproc.GaussianBlur(mat, mat, new Size(17, 17), 8);

					//Identify the circles
					Imgproc.HoughCircles(mat, mat, Imgproc.HOUGH_GRADIENT, 1.2, 100, 100, 30, 75, 400);

					//Put the values to Smart Dashboard
					if (mat.rows() > 0) {
						SmartDashboard.putNumber("Smart Ball X", Math.atan(mat.get(0, 0)[0] - 320));
						SmartDashboard.putNumber("Smart Ball y", Math.atan(mat.get(0, 0)[1] - 240));
						SmartDashboard.putBoolean("Ball in range", mat.get(0, 0)[2] < Constants.CVConstants.kBallRadiusForIntake);
					} else {
						SmartDashboard.putNumber("Smart Ball X", 0);
						SmartDashboard.putNumber("Smart Ball y", 0);
					}
				}
			}
		};

		m_visionThread.start();
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for
	 * items like diagnostics that you want ran during disabled, autonomous,
	 * teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
	}
}
