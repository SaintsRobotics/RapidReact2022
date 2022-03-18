package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;

public class test {
    public static void main(String[] args) {
        //Set up the camera server
		UsbCamera camera = CameraServer.startAutomaticCapture();

		//Set the resolution
		camera.setResolution(640, 480);

		//Set up the Sink for the video
		CvSink cvSink = CameraServer.getVideo();

		//Create the matrix for the image
		Mat mat = new Mat();

        //Create the vars
        double x, y;

		//Create the parallel thread that manages video capturing.
		while (true) {

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

            x = mat.get(0, 0)[0];
            y = mat.get(0, 1)[1];

		System.out.print(x);
        System.out.print(" ");
        System.out.println(y);
        
		}
    }	
}

