package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

/**
 * Limelight subsystem for AprilTag detection and tracking
 */
public class Limelight {

    private Limelight3A limelight;
    private LLResult latestResult;

    // Shooter TPS mapping based on ty (distance)
    // ty decreases as distance increases (high ty = close, low ty = far)
    public static final double LIMELIGHT_TY_1 = 14.3;   // Closest
    public static final double LIMELIGHT_TY_2 = 7.1;
    public static final double LIMELIGHT_TY_3 = 1.7;
    public static final double LIMELIGHT_TY_4 = 2.2;
    public static final double LIMELIGHT_TY_5 = -0.5;
    public static final double LIMELIGHT_TY_6 = -1.8;   // Farthest

    public static final double SHOOTER_TPS_1 = 1350.0;  // Closest
    public static final double SHOOTER_TPS_2 = 1450.0;
    public static final double SHOOTER_TPS_3 = 1510.0;
    public static final double SHOOTER_TPS_4 = 1650.0;
    public static final double SHOOTER_TPS_5 = 1750.0;
    public static final double SHOOTER_TPS_6 = 1800.0;  // Farthest

    public static final double SHOOTER_MIN_TPS = 1250.0;
    public static final double SHOOTER_MAX_TPS = 1900;
    public static final double SHOOTER_DEFAULT_TPS = 1750.0;

    // Alignment thresholds
    public static final double ALIGNMENT_TOLERANCE_CLOSE = 2;
    public static final double ALIGNMENT_TOLERANCE_FAR = 5.0;
    public static final double AREA_CLOSE_THRESHOLD = 0.5;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    /**
     * Start the Limelight
     */
    public void start() {
        limelight.start();
    }

    /**
     * Switch to a specific pipeline
     */
    public void switchPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    /**
     * Update the latest result from Limelight
     */
    public void update() {
        latestResult = limelight.getLatestResult();
    }

    /**
     * Check if Limelight has a valid target
     */
    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid();
    }

    /**
     * Get tx (horizontal offset to target)
     */
    public double getTx() {
        if (latestResult != null && latestResult.isValid()) {
            return latestResult.getTx();
        }
        return 0.0;
    }

    /**
     * Get ty (vertical offset to target)
     */
    public double getTy() {
        if (latestResult != null && latestResult.isValid()) {
            return latestResult.getTy();
        }
        return 0.0;
    }

    /**
     * Get ta (target area)
     */
    public double getTa() {
        if (latestResult != null && latestResult.isValid()) {
            return latestResult.getTa();
        }
        return 0.0;
    }

    /**
     * Get the latest result (for advanced usage)
     */
    public LLResult getLatestResult() {
        return latestResult;
    }

    /**
     * Get the detected AprilTag ID
     */
    public int getAprilTagId() {
        if (latestResult != null && latestResult.isValid()) {
            if (latestResult.getFiducialResults() != null && !latestResult.getFiducialResults().isEmpty()) {
                return (int) latestResult.getFiducialResults().get(0).getFiducialId();
            }
        }
        return -1;
    }

    /**
     * Get target shooter TPS based on ty (distance)
     */
    public double getTargetShooterTPS() {
        if (!hasTarget()) {
            return SHOOTER_DEFAULT_TPS;
        }

        double ty = getTy();
        double targetTPS;

        // ty decreases as distance increases (high ty = close, low ty = far)
        if (ty >= LIMELIGHT_TY_1) {
            targetTPS = SHOOTER_TPS_1;
        } else if (ty >= LIMELIGHT_TY_2) {
            double tyRange = LIMELIGHT_TY_1 - LIMELIGHT_TY_2;
            double tpsRange = SHOOTER_TPS_2 - SHOOTER_TPS_1;
            double normalized = (LIMELIGHT_TY_1 - ty) / tyRange;
            targetTPS = SHOOTER_TPS_1 + (normalized * tpsRange);
        } else if (ty >= LIMELIGHT_TY_3) {
            double tyRange = LIMELIGHT_TY_2 - LIMELIGHT_TY_3;
            double tpsRange = SHOOTER_TPS_3 - SHOOTER_TPS_2;
            double normalized = (LIMELIGHT_TY_2 - ty) / tyRange;
            targetTPS = SHOOTER_TPS_2 + (normalized * tpsRange);
        } else if (ty >= LIMELIGHT_TY_4) {
            double tyRange = LIMELIGHT_TY_3 - LIMELIGHT_TY_4;
            double tpsRange = SHOOTER_TPS_4 - SHOOTER_TPS_3;
            double normalized = (LIMELIGHT_TY_3 - ty) / tyRange;
            targetTPS = SHOOTER_TPS_3 + (normalized * tpsRange);
        } else if (ty >= LIMELIGHT_TY_5) {
            double tyRange = LIMELIGHT_TY_4 - LIMELIGHT_TY_5;
            double tpsRange = SHOOTER_TPS_5 - SHOOTER_TPS_4;
            double normalized = (LIMELIGHT_TY_4 - ty) / tyRange;
            targetTPS = SHOOTER_TPS_4 + (normalized * tpsRange);
        } else if (ty >= LIMELIGHT_TY_6) {
            double tyRange = LIMELIGHT_TY_5 - LIMELIGHT_TY_6;
            double tpsRange = SHOOTER_TPS_6 - SHOOTER_TPS_5;
            double normalized = (LIMELIGHT_TY_5 - ty) / tyRange;
            targetTPS = SHOOTER_TPS_5 + (normalized * tpsRange);
        } else {
            targetTPS = SHOOTER_TPS_6;
        }

        // Clamp to valid range
        if (targetTPS < SHOOTER_MIN_TPS) targetTPS = SHOOTER_MIN_TPS;
        if (targetTPS > SHOOTER_MAX_TPS) targetTPS = SHOOTER_MAX_TPS;

        return targetTPS;
    }

    /**
     * Check if aligned for shooting (tx within tolerance)
     */
    public boolean isAlignedForShooting() {
        if (!hasTarget()) {
            return false; // No target = not aligned, wait for AprilTag
        }
        double tx = Math.abs(getTx());
        double area = getTa();
        double tolerance = (area >= AREA_CLOSE_THRESHOLD) ? ALIGNMENT_TOLERANCE_CLOSE : ALIGNMENT_TOLERANCE_FAR;
        return tx <= tolerance;
    }
}
