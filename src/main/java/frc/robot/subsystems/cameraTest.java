package frc.robot.subsystems;

import java.io.File;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;

public class cameraTest {

    public static void main(String[] args) {
       //Set up the camera server
		UsbCamera camera = CameraServer.startAutomaticCapture();

		//Set the resolution
		camera.setResolution(640, 480);

		//Set up the Sink for the video
		CvSink cvSink = CameraServer.getVideo();

		//Create the matrix for the image
		//Mat mat = new Mat();

        //moment of truth
        //System.out.println(cvSink.grabFrame(mat));
    }
    
}
