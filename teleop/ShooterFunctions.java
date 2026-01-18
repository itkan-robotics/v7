package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

/**
 * Shooter motor control and speed management
 * Self-contained with its own hardware initialization
 */
public class ShooterFunctions {
    
    // ========== HARDWARE ==========
    private DcMotorEx shooterMotor;
    private Limelight3A limelight;
    
    // ========== CONSTANTS ==========
    // Shooter settings (ticks per second)
    public static final double SHOOTER_DEFAULT_TPS = 1750.0;
    public static final double SHOOTER_MIN_TPS = 1250.0;
    public static final double SHOOTER_MAX_TPS = 2000.0;
    public static final double SHOOTER_READY_THRESHOLD = 50.0;
    public static final double SHOOTER_MAX_POWER = 1.0;
    
    // Limelight ty to shooter TPS mapping - values come from RobotConstants
    
    // Alignment tolerance thresholds
    public static final double APRILTAG_AREA_CLOSE_THRESHOLD = 0.5;
    public static final double SHOOTER_READY_ALIGNMENT_TOLERANCE_CLOSE = 10.0;
    public static final double SHOOTER_READY_ALIGNMENT_TOLERANCE_FAR = 5.0;
    
    // ========== STATE ==========
    private double lastShooterPower = 0.0;
    private double overrideDefaultTPS = 0.0;  // When > 0, overrides SHOOTER_DEFAULT_TPS
    
    public ShooterFunctions(HardwareMap hardwareMap, Limelight3A sharedLimelight) {
        // Initialize shooter motor
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooterMotor.setDirection(RobotConstants.getShooterDirection());
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Use shared Limelight (initialized elsewhere since multiple systems use it)
        this.limelight = sharedLimelight;
    }
    
    /**
     * Re-apply motor settings based on current RobotConstants.
     * Call this after changing the robot selection.
     */
    public void applyMotorSettings() {
        shooterMotor.setDirection(RobotConstants.getShooterDirection());
    }
    
    // ========== SHOOTER CONTROL ==========
    
    public double getShooterTPS() {
        return Math.abs(shooterMotor.getVelocity());
    }
    
    public double getShooterRPM() {
        final double TICKS_PER_REV = 28.0;
        double tps = getShooterTPS();
        return (tps * 60.0) / TICKS_PER_REV;
    }
    
    /**
     * Get target TPS using a cached ty value (call this after TurretFunctions.updateLimelightData())
     * @param cachedTy The cached ty value from TurretFunctions
     * @param hasTarget Whether we have a valid limelight target
     */
    public double getTargetShooterTPS(double cachedTy, boolean hasTarget) {
        if (!hasTarget) {
            // Use override default if set, otherwise use constant default
            return (overrideDefaultTPS > 0) ? overrideDefaultTPS : SHOOTER_DEFAULT_TPS;
        }
        
        double ty = cachedTy;
        double targetTPS;
        
        // Get robot-specific constants
        double TY_1 = RobotConstants.getLimelightTy1();
        double TY_2 = RobotConstants.getLimelightTy2();
        double TY_3 = RobotConstants.getLimelightTy3();
        double TY_4 = RobotConstants.getLimelightTy4();
        double TY_5 = RobotConstants.getLimelightTy5();
        double TY_6 = RobotConstants.getLimelightTy6();
        double TPS_1 = RobotConstants.getShooterTps1();
        double TPS_2 = RobotConstants.getShooterTps2();
        double TPS_3 = RobotConstants.getShooterTps3();
        double TPS_4 = RobotConstants.getShooterTps4();
        double TPS_5 = RobotConstants.getShooterTps5();
        double TPS_6 = RobotConstants.getShooterTps6();
        
        // ty decreases as distance increases (high ty = close, low ty = far)
        if (ty >= TY_1) {
            // Closer than closest calibration point
            targetTPS = TPS_1;
        } else if (ty >= TY_2) {
            // Interpolate between TY_1 and TY_2
            double tyRange = TY_1 - TY_2;
            double tpsRange = TPS_2 - TPS_1;
            double normalized = (TY_1 - ty) / tyRange;
            targetTPS = TPS_1 + (normalized * tpsRange);
        } else if (ty >= TY_3) {
            double tyRange = TY_2 - TY_3;
            double tpsRange = TPS_3 - TPS_2;
            double normalized = (TY_2 - ty) / tyRange;
            targetTPS = TPS_2 + (normalized * tpsRange);
        } else if (ty >= TY_4) {
            double tyRange = TY_3 - TY_4;
            double tpsRange = TPS_4 - TPS_3;
            double normalized = (TY_3 - ty) / tyRange;
            targetTPS = TPS_3 + (normalized * tpsRange);
        } else if (ty >= TY_5) {
            double tyRange = TY_4 - TY_5;
            double tpsRange = TPS_5 - TPS_4;
            double normalized = (TY_4 - ty) / tyRange;
            targetTPS = TPS_4 + (normalized * tpsRange);
        } else if (ty >= TY_6) {
            double tyRange = TY_5 - TY_6;
            double tpsRange = TPS_6 - TPS_5;
            double normalized = (TY_5 - ty) / tyRange;
            targetTPS = TPS_5 + (normalized * tpsRange);
        } else {
            // Farther than farthest calibration point
            targetTPS = TPS_6;
        }
        
        if (targetTPS < SHOOTER_MIN_TPS) targetTPS = SHOOTER_MIN_TPS;
        if (targetTPS > SHOOTER_MAX_TPS) targetTPS = SHOOTER_MAX_TPS;
        
        return targetTPS;
    }
    
    /**
     * Get target TPS by reading limelight directly (for standalone use or backwards compatibility)
     * Prefer using getTargetShooterTPS(cachedTy, hasTarget) with cached data.
     */
    public double getTargetShooterTPS() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return (overrideDefaultTPS > 0) ? overrideDefaultTPS : SHOOTER_DEFAULT_TPS;
        }
        return getTargetShooterTPS(result.getTy(), true);
    }
    
    public void controlShooter(boolean running) {
        controlShooter(running, false);
    }
    
    public void controlShooter(boolean running, boolean isActivelyFeeding) {
        if (!running) {
            shooterMotor.setPower(0);
            lastShooterPower = 0.0;
            return;
        }
        
        double targetTPS = getTargetShooterTPS();
        double currentTPS = getShooterTPS();
        
        // For far shots (high RPM), increase target by 100 TPS while actively feeding
        // to compensate for RPM drop during shooting
        if (isActivelyFeeding && targetTPS > 1700) {
            targetTPS += 0;
        }
        
        double power = (currentTPS < targetTPS) ? SHOOTER_MAX_POWER : 0.0;
        
        shooterMotor.setPower(power);
        lastShooterPower = power;
    }
    
    public void controlShooterManual(double targetTPS, boolean running) {
        controlShooterManual(targetTPS, running, false);
    }
    
    public void controlShooterManual(double targetTPS, boolean running, boolean isActivelyFeeding) {
        if (!running) {
            shooterMotor.setPower(0);
            lastShooterPower = 0.0;
            return;
        }
        
        double currentTPS = getShooterTPS();
        
        // For far shots (high RPM), increase target by 100 TPS while actively feeding
        double effectiveTarget = targetTPS;
        if (isActivelyFeeding && targetTPS > 1700) {
            effectiveTarget += 100;
        }
        
        double power = (currentTPS < effectiveTarget) ? SHOOTER_MAX_POWER : 0.0;
        
        shooterMotor.setPower(power);
        lastShooterPower = power;
    }
    
    public void stopShooter() {
        shooterMotor.setPower(0);
        lastShooterPower = 0.0;
    }
    
    public double getShooterPower() {
        return lastShooterPower;
    }
    
    // ========== DEFAULT TPS OVERRIDE ==========
    
    /**
     * Set a temporary override for the default TPS (used when no AprilTag visible)
     * When AprilTag IS visible, ty-based TPS is still used
     */
    public void setDefaultTPSOverride(double tps) {
        overrideDefaultTPS = tps;
    }
    
    /**
     * Clear the default TPS override
     */
    public void clearDefaultTPSOverride() {
        overrideDefaultTPS = 0.0;
    }
    
    /**
     * Get the current default TPS (override if set, otherwise constant)
     */
    public double getCurrentDefaultTPS() {
        return (overrideDefaultTPS > 0) ? overrideDefaultTPS : SHOOTER_DEFAULT_TPS;
    }
    
    public boolean isShooterSpeedReady() {
        double targetTPS = getTargetShooterTPS();
        double currentTPS = getShooterTPS();
        return Math.abs(currentTPS - targetTPS) <= SHOOTER_READY_THRESHOLD;
    }
    
    public boolean isShooterSpeedReady(double targetTPS) {
        double currentTPS = getShooterTPS();
        return Math.abs(currentTPS - targetTPS) <= SHOOTER_READY_THRESHOLD;
    }
    
    public boolean isShooterReady(boolean turretOnTarget) {
        return isShooterSpeedReady() && turretOnTarget;
    }
    
    public double getAlignmentTolerance(double area) {
        if (area >= APRILTAG_AREA_CLOSE_THRESHOLD) {
            return SHOOTER_READY_ALIGNMENT_TOLERANCE_CLOSE;
        } else {
            return SHOOTER_READY_ALIGNMENT_TOLERANCE_FAR;
        }
    }
}
