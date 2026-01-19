package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Static class to persist robot pose between OpModes.
 * Auto updates this as it runs, TeleOp reads it at startup.
 * 
 * Note: Static variables persist as long as the app is running.
 * A phone restart or app restart will reset these values.
 */
public class PoseStorage {
    
    // Last known robot position (persists between OpModes)
    public static double x = 0;
    public static double y = 0;
    public static double heading = 0;
    
    // Flag to indicate if pose has been set (vs default 0,0,0)
    private static boolean poseSet = false;
    
    // Alliance (persists between OpModes)
    private static boolean isRedAlliance = true;
    
    /**
     * Save the current robot pose (call this from auto periodically)
     */
    public static void savePose(double xInches, double yInches, double headingDegrees) {
        x = xInches;
        y = yInches;
        heading = headingDegrees;
        poseSet = true;
    }
    
    /**
     * Save pose from a Pose2D object
     */
    public static void savePose(Pose2D pose) {
        x = pose.getX(DistanceUnit.INCH);
        y = pose.getY(DistanceUnit.INCH);
        heading = pose.getHeading(AngleUnit.DEGREES);
        poseSet = true;
    }
    
    /**
     * Get the saved pose as a Pose2D
     */
    public static Pose2D getPose() {
        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
    }
    
    /**
     * Get X position in inches
     */
    public static double getX() {
        return x;
    }
    
    /**
     * Get Y position in inches
     */
    public static double getY() {
        return y;
    }
    
    /**
     * Get heading in degrees
     */
    public static double getHeading() {
        return heading;
    }
    
    /**
     * Check if a pose has been saved (vs using default)
     */
    public static boolean hasSavedPose() {
        return poseSet;
    }
    
    /**
     * Clear the saved pose (reset to default)
     */
    public static void clearPose() {
        x = 0;
        y = 0;
        heading = 0;
        poseSet = false;
    }
    
    /**
     * Save the alliance selection
     */
    public static void setAlliance(boolean isRed) {
        isRedAlliance = isRed;
    }
    
    /**
     * Get the saved alliance
     */
    public static boolean isRedAlliance() {
        return isRedAlliance;
    }
}
