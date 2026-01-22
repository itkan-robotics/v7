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
    
    // ========== UNIT CONVERSIONS ==========
    public static final double INCHES_TO_MM = 25.4;
    public static final double MM_TO_INCHES = 1.0 / 25.4;
    
    // ========== GOAL POSITIONS (mm) ==========
    public static final double GOAL_RED_X = 138.0 * INCHES_TO_MM;
    public static final double GOAL_RED_Y = 143.5 * INCHES_TO_MM;
    public static final double GOAL_BLUE_X = 6.0 * INCHES_TO_MM;
    public static final double GOAL_BLUE_Y = 143.5 * INCHES_TO_MM;
    
    // ========== DRIVETRAIN CONSTANTS ==========
    public static final double COUNTS_PER_MOTOR_REV = 537.7;
    public static final double WHEEL_DIAMETER_MM = 96.0;
    public static final double COUNTS_PER_MM = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_MM);
    public static final double DRIVE_HEADING_TOLERANCE = 2.0;
    
    // Field positions (inches)
    public static final double FIELD_CENTER_X_INCHES = 72;
    public static final double FIELD_CENTER_Y_INCHES = 72;
    
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
    
    // ========== TURRET MOTOR CONSTANTS ==========
    // Turret motor ticks per full rotation (depends on motor + gearing)
    public static final double TURRET_TICKS_PER_REV = 373;
    public static final double TURRET_TICKS_PER_DEGREE = (TURRET_TICKS_PER_REV) / 355.0;
    
    // Turret hardstop deadzone: usable range is 5° to 355° (350° total rotation)
    // 0 ticks corresponds to 5°, max ticks corresponds to 355°
    public static final double TURRET_MIN_ANGLE = 5.0;        // Angle at 0 ticks (hardstop)
    public static final double TURRET_MAX_ANGLE = 355.0;      // Angle at max ticks (hardstop)
    public static final double TURRET_ANGLE_RANGE = TURRET_MAX_ANGLE - TURRET_MIN_ANGLE;  // 350°
    public static final double TURRET_CENTER_OFFSET = 2.5 * INCHES_TO_MM;
    
    // Turret range limits (in ticks)
    public static final double TURRET_MIN_TICKS = 0;          // Corresponds to 5°
    public static final double TURRET_MAX_TICKS = 373;  // Corresponds to 355°
    
    // Turret PID constants
    //public static final double TURRET_KP = 0.002;

    public static final double TURRET_KP = 0.05;

    public static final double TURRET_KI = 0.0;
    public static final double TURRET_KD = 0.0;

    // Visual tracking PD
    public static final double TURRET_VISUAL_KP = 0.003;
    public static final double TURRET_VISUAL_KD = 0.01;
    
    // Turret feedforward to counteract base rotation
    public static final double TURRET_TURN_FF = -0.9;
    public static final double TURRET_VISUAL_KF = 0.15;
    
    // Legacy turret servo constants (for reference)
    public static double getTurretHeadingSign() {
        return 1.0;  // Same for both robots
    }
    
    public static double getTurretFieldOffset() {
        return 0.0;  // Same for both robots
    }
    
    public static boolean hasDualTurretServos() {
        return false;  // Both robots use motor turret now
    }
    
    public static double getTurretKpFar() {
        return 0.0001;
    }
    
    public static double getTurretKpClose() {
        return 0.00015;
    }
    
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
    public static final double SHOOTER_MAX_POWER = 1.0;
    public static final double SHOOTER_DEFAULT_TPS = 1750.0;
    public static final double SHOOTER_MIN_TPS = 1250.0;
    public static final double SHOOTER_MAX_TPS = 2000.0;
    public static final double SHOOTER_READY_THRESHOLD = 50.0;
    public static final double DEFAULT_TARGET_SHOOTER_VELOCITY = 1350.0;
    public static final double VELOCITY_TOLERANCE = 50.0;
    
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
    
    // ========== INTAKE CONSTANTS ==========
    public static final double INTAKE_POWER = 1.0;
    public static final double THREE_BALL_POWER_THRESHOLD = 62.0;
    public static final double TRANSFER_STARTUP_IGNORE_TIME = 500;
    public static final double INTAKE_CURRENT_THRESHOLD = 2.0;
    public static final double INTAKE_VELOCITY_THRESHOLD = 50.0;
    public static final double INTAKE_STALL_VELOCITY = 10.0;
    public static final double INTAKE_POWER_THRESHOLD = 0.8;
    
    // ========== BLOCKER CONSTANTS ==========
    
    public static double getBlockerBlocked() {
        return is21171() ? 0.35 : 0.3;
    }
    
    public static double getBlockerUnblocked() {
        return is21171() ? 0.65 : 0.55;
    }
    
    // ========== INDEXER CONSTANTS ==========
    public static final double INDEXER_INDEXED = 0.05;
    public static final double INDEXER_MIDDLE = 0.55;
    
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
    
    // ========== LED CONSTANTS ==========
    public static final double LIGHT_OFF = 0.0;
    public static final double LIGHT_RED = 0.305;
    public static final double LIGHT_ORANGE = 0.333;
    public static final double LIGHT_YELLOW = 0.388;
    public static final double LIGHT_GREEN = 0.5;
    public static final double LIGHT_BLUE = 0.666;
    public static final double LIGHT_PURPLE = 0.722;
    public static final double LIGHT_WHITE = 1.0;
    
    // ========== LIMELIGHT CONSTANTS ==========
    public static final double LIMELIGHT_TOLERANCE = 1.5;
    public static final double LIMELIGHT_KP = 0.035;
}

