// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;

/** Class to access the limelight through network tables. */
public final class Limelight {
    /**
     * Returns the vertical offset from crosshair to target.
     * 
     * @return Vertical offset from -24.85 to 24.85 degrees.
     */
    public static double getY() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    }

    /**
     * Returns the horizontal offset from crosshair to target.
     * 
     * @return Horizontal offset from -29.8 to 29.8 degrees.
     */
    public static double getX() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }

    /**
     * Returns the angle of the target.
     * 
     * @return Angle of the target from -90 to 0 degrees.
     */
    public static double getAngle() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
    }

    /**
     * Returns the pipeline’s latency contribution.
     * 
     * @return Latency value in ms.
     */
    public static double getLatency() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0);
    }

    /**
     * Returns the area of the target.
     * 
     * @return Area value from 0-100% of image.
     */
    public static double getArea() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    }

    /**
     * Returns if the limelight has any valid targets.
     * 
     * @return If the limelight has a target.
     */
    public static boolean hasTarget() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 0 ? false : true;
    }

    /**
     * Sets limelight’s LED state.
     * <p>
     * 0: Use LED Mode set in the current pipeline.
     * <p>
     * 1: Force off.
     * <p>
     * 2: Force blink.
     * <p>
     * 3: Force on.
     * 
     * @param state Limelight LED state.
     */
    public static void setLed(double state) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(state);
    }

    /**
     * Sets limelight’s current pipeline.
     * <p>
     * 0: Targets shooting target.
     * <p>
     * 1: Targets blue ball.
     * <p>
     * 2: Targets red ball.
     * 
     * @param pipeline Select pipeline 0-9.
     */
    public static void setPipeline(double pipeline) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
    }
}
