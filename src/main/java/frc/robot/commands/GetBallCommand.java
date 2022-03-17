// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;

/** Uses the {@link Limelight} to intake a ball belonging to our alliance. */
public class GetBallCommand extends CommandBase {

	// TODO: tune pid
	private final PIDController m_rotPid = new PIDController(0.03, 0, 0);
	private NetworkTable table;
	private UsbCamera camera;
	private VideoMode videoMode;
	private MoveCommand m_moveCommand;
	private final int m_pipeline;
	private static double COLLECTION_TOLERANCE = 0.45;
	private static double MOUNTING_ANGLE_DEGREES = -10;
	private static double LIMELIGHT_HEIGHT = 0.5;
	private static double BALL_CENTER_HEIGHT = 0.1;


	/**
	 * Creates a new {@link GetBallCommand}.
	 * 
	 * @param subsystem The required subsystem.
	 * @param pipeline  Pipeline index to aim at the ball with.
	 */
	public GetBallCommand(MoveCommand moveCommand, int pipeline) {
		m_moveCommand = moveCommand;
		m_pipeline = pipeline;
		m_rotPid.setTolerance(0.1);

		//image related stuff
		NetworkTableInstance.getDefault().startClient(); //
		table = NetworkTableInstance.getDefault().getTable("limelight");
		
		// usb camera
		camera = CameraServer.startAutomaticCapture();
		//camera.setResolution(640, 360); //only works for 640 x 360
		videoMode = new VideoMode(PixelFormat.kYUYV, 800, 448, 30);
		//set DriverStation resolution to:
		//320 x 240 for ~15 fps
		//160 x 120 for ~20 fps
		camera.setFPS(30);
		camera.setVideoMode(videoMode);

	}

	@Override
	public void initialize() {
		Limelight.setPipeline(m_pipeline);
		Limelight.setLED(0);
		Limelight.setCameraMode(0);

		double distance = getDistance();

		m_moveCommand.withRotSpeedSupplier(() -> m_rotPid.calculate(Limelight.getX(), 0)).withRobotRelativeX(distance)
				.schedule();
	}

	@Override
	public void end(boolean interrupted) {
		Limelight.setLED(1);
		Limelight.setCameraMode(1);
		m_moveCommand.cancel();
	}

	@Override
	public boolean isFinished() {
		return Math.abs(getDistance()) < COLLECTION_TOLERANCE
				&& Math.abs(m_rotPid.calculate(SmartDashboard.getNumber("Smart Ball x", 0), 0)) < 0.03;
	}

	public double getDistance() {
		return (BALL_CENTER_HEIGHT - LIMELIGHT_HEIGHT)
				/ (Math.tan((MOUNTING_ANGLE_DEGREES + SmartDashboard.getNumber("Smart Ball y", 0)) * Math.PI / 180)); // units in
		// meters,
		// converted
		// from degrees
		// to radians
	}

	private void getBallImage() {
		table.getEntry("filename");		
	}
}
