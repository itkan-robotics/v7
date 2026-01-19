package org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Centralized robot-specific constants for both 21171 and 19564 robots.
 * Select the robot at runtime to use the correct constants.
 */
public class RobotConstants {
    
    // ========== ROBOT IDENTIFICATION ==========
    public static final int ROBOT_21171 = 21171;
    public static final int ROBOT_19564 = 19564;
    
    // Current robot selection (set during init)
    private static int currentRobot = ROBOT_19564;
    
    public static void setRobot(int robotId) {
        currentRobot = robotId;
    }
    
    public static int getCurrentRobot() {
        return currentRobot;
    }
    
    public static boolean is21171() {
        return currentRobot == ROBOT_21171;
    }
    
    public static boolean is19564() {
        return currentRobot == ROBOT_19564;
    }
    
    // ========== DRIVETRAIN CONSTANTS ==========
    
    public static GoBildaPinpointDriver.EncoderDirection getEncoderDirectionX() {
        return is21171() 
            ? GoBildaPinpointDriver.EncoderDirection.REVERSED 
            : GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }
    
    public static GoBildaPinpointDriver.EncoderDirection getEncoderDirectionY() {
        return GoBildaPinpointDriver.EncoderDirection.REVERSED;  // Same for both
    }
    
    public static double getYawScalar() {
        return -1.0;  // Same for both robots
    }
    
    /**
     * Get the sign multiplier for heading in turret angle calculation.
     * Now same for both robots since yaw scalar is the same.
     */
    public static double getTurretHeadingSign() {
        return 1.0;  // Same for both robots
    }
    
    /**
     * Get the turret field offset in degrees.
     * Same for both robots now that yaw scalar is unified.
     */
    public static double getTurretFieldOffset() {
        return 0.0;  // Same for both robots
    }
    
    // ========== TURRET CONSTANTS ==========
    
    public static boolean hasDualTurretServos() {
        return false;  // Both robots have 1 turret servo
    }
    
    // Turret KP values (same for both robots)
    public static double getTurretKpFar() {
        return 0.0001;
    }
    
    public static double getTurretKpClose() {
        return 0.00015;
    }
    
    // Turret tolerance values (same for both robots)
    public static double getTurretToleranceClose() {
        return 5.0;
    }
    
    public static double getTurretToleranceCloseDist() {
        return 40.0;
    }
    
    public static boolean hasFarShotTxOffset() {
        return is21171();
    }
    
    // ========== PINPOINT ODOMETRY OFFSETS ==========
    
    /**
     * Get pinpoint X offset in mm (distance from center to X encoder)
     */
    public static double getPinpointOffsetX() {
        return is21171() ? -55.0 : -55.0;  // TODO: Update 19564 value if different
    }
    
    /**
     * Get pinpoint Y offset in mm (distance from center to Y encoder)
     */
    public static double getPinpointOffsetY() {
        return is21171() ? -12.0 : -12.0;  // TODO: Update 19564 value if different
    }
    
    // ========== SHOOTER CONSTANTS ==========
    
    public static DcMotorSimple.Direction getShooterDirection() {
        return is21171() 
            ? DcMotorSimple.Direction.REVERSE 
            : DcMotorSimple.Direction.FORWARD;
    }
    
    // Limelight ty to shooter TPS mapping
    public static double getLimelightTy1() { return 14.3; }  // Same for both
    
    public static double getLimelightTy2() {
        return is21171() ? 2.4 : 7.1;
    }
    
    public static double getLimelightTy3() {
        return is21171() ? -0.4 : 1.7;
    }
    
    public static double getLimelightTy4() {
        return is21171() ? -3.2 : 2.2;
    }
    
    public static double getLimelightTy5() {
        return is21171() ? -4.2 : -0.5;
    }
    
    public static double getLimelightTy6() {
        return is21171() ? -4.8 : -1.8;
    }
    
    public static double getShooterTps1() { return 1350.0; }  // Same for both
    public static double getShooterTps2() { return 1450.0; }  // Same for both
    public static double getShooterTps3() { return 1500.0; }  // Same for both
    
    public static double getShooterTps4() {
        return is21171() ? 1750.0 : 1600.0;
    }
    
    public static double getShooterTps5() {
        return is21171() ? 1800.0 : 1750.0;
    }
    
    public static double getShooterTps6() { return 1800.0; }  // Same for both
    
    // ========== INTAKE/BLOCKER CONSTANTS ==========
    
    public static double getBlockerBlocked() {
        return is21171() ? 0.35 : 0.3;
    }
    
    public static double getBlockerUnblocked() {
        return is21171() ? 0.65 : 0.55;
    }
    
    // ========== INDEXER CONSTANTS ==========
    
    public static double getIndexerIndexed() {
        return is21171() ? 0.45 : 0.05;
    }
    
    public static double getIndexerMiddle() {
        return is21171() ? 0.925 : 0.55;
    }
    
    // ========== CLIMBER CONSTANTS ==========
    
    public static double getClimberDown() {
        return is21171() ? 0.15 : 0.6;
    }
    
    public static double getClimberUp() {
        return is21171() ? 0.95 : 0.0;
    }
}

