package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.HashMap;
import java.util.Map;
import java.util.List;

public class Limelight {
    Limelight3A limelight;

    // Constants for limelight subsystem
    public static final int DEFAULT_PIPELINE = 0;
    
    // Shooter velocity calculation constants
    public static final double MIN_SHOOTER_VELOCITY = 1250.0;
    public static final double MAX_SHOOTER_VELOCITY = 2000.0;
    public static final double DEFAULT_TARGET_SHOOTER_VELOCITY = 1350.0;
    
    // Limelight area to shooter TPS mapping (calibrated values from testing)
    public static final double LIMELIGHT_AREA_1 = 4.3;    // Closest
    public static final double LIMELIGHT_AREA_2 = 1.5;    
    public static final double LIMELIGHT_AREA_3 = 0.88;   
    public static final double LIMELIGHT_AREA_4 = 0.7;    
    public static final double LIMELIGHT_AREA_5 = 0.4;    
    public static final double LIMELIGHT_AREA_6 = 0.3;    // Farthest
    public static final double SHOOTER_TPS_1 = 1350.0;    // TPS at 4.3%
    public static final double SHOOTER_TPS_2 = 1400.0;    // TPS at 1.5%
    public static final double SHOOTER_TPS_3 = 1500.0;    // TPS at 0.88%
    public static final double SHOOTER_TPS_4 = 1600.0;    // TPS at 0.7%
    public static final double SHOOTER_TPS_5 = 1700.0;    // TPS at 0.4%
    public static final double SHOOTER_TPS_6 = 1850.0;    // TPS at 0.3%
    
    // Target velocity calculation parameters
    public static final double MIN_TY = -3.5;  // ty at max shooting distance
    public static final double MAX_TY = 11;    // ty at min shooting distance
    public static final double TY_POWER = 0.43; // Power for ty normalization
    public static final double VELOCITY_OFFSET = -50; // Reduced from 100 to lower overall velocity
    
    // Aim assist constants
    public static final double DEFAULT_KP = 0.02;
    public static final double DEFAULT_KD = 0.005; // Derivative coefficient for damping
    public static final double DEFAULT_KF = 0.075;   // Feedforward coefficient
    public static final double MAX_DRIVE_POWER = 0.5;
    public static final double MIN_DRIVE_POWER = 0.1;
    public static final double TARGET_TX_THRESHOLD = 2.0;
    public static final double TARGET_TX_BLUE = 1.5;
    public static final double TARGET_TX_RED = -1.5;
    public static final double TARGET_TX_DEFAULT = 0.5;
    public static final double TARGET_AREA_THRESHOLD = 0.5;
    
    // Alignment tolerance for shooting (based on distance)
    // IMPORTANT: These must be <= LIMELIGHT_TOLERANCE in Shooter.java (2.0) to prevent shooting before turret is aligned
    public static final double SHOOTER_READY_ALIGNMENT_TOLERANCE_CLOSE = 2.0;  // Degrees tolerance when close (area >= 0.5)
    public static final double SHOOTER_READY_ALIGNMENT_TOLERANCE_FAR = 2.0;    // Degrees tolerance when far (area < 0.5)
    
    // Limelight 3A FOV constants (default specifications)
    // These are the standard field of view values for Limelight 3A
    public static final double LIMELIGHT_HORIZONTAL_FOV = 59.6; // degrees - horizontal field of view
    public static final double LIMELIGHT_VERTICAL_FOV = 45.7; // degrees - vertical field of view
    
    // Limelight detection range constants (default values, may need tuning)
    // Typical AprilTag detection range: 1-2 feet minimum, up to 10-15 feet maximum
    public static final double LIMELIGHT_MAX_RANGE = 180.0; // inches (15 feet) - maximum reliable detection range
    public static final double LIMELIGHT_MIN_RANGE = 12.0; // inches (1 foot) - minimum reliable detection range
    public static final double OPTIMAL_SHOOTING_DISTANCE = 60.0; // inches - preferred distance for shooting
    
    // AprilTag field positions (in inches, FTC field coordinate system)
    // Field coordinate system: Origin at center of field, X+ = right, Y+ = forward from driver station
    // These are default positions - verify with actual field measurements for your season
    // For FTC 2024-2025 season (Centerstage), backboard AprilTags are typically:
    // - Tag 20: Blue alliance backboard center
    // - Tag 24: Red alliance backboard center
    // Default positions assume standard field layout - UPDATE WITH ACTUAL FIELD MEASUREMENTS
    private static final Map<Integer, Pose2D> APRILTAG_POSITIONS = new HashMap<>();

    LLResult currentResult;
    
    static {
        // Initialize AprilTag positions (from HardwareConfigAuto, converted from mm to inches)
        // HardwareConfigAuto uses mm, Limelight uses inches
        // Tag 24 = Red alliance backboard center
        // Position from HardwareConfigAuto: (0.0 mm, 1981.2 mm) = (0.0 in, 78.0 in)
        // Field coordinate system: X=0 at center, Y=1981.2mm (78in) left from center
        // Converting to FTC field coordinates (origin at corner): approximately (72, 150) inches
        // Using approximate conversion: tag is 78in from center, field is 144in wide
        APRILTAG_POSITIONS.put(24, new Pose2D(DistanceUnit.INCH, 72.0, 150.0, AngleUnit.DEGREES, 0));
        
        // Tag 20 = Blue alliance backboard center
        // Position from HardwareConfigAuto: (0.0 mm, -1981.2 mm) = (0.0 in, -78.0 in)
        // Converting to FTC field coordinates: approximately (72, -6) inches, but typically mirrored
        APRILTAG_POSITIONS.put(20, new Pose2D(DistanceUnit.INCH, 72.0, -6.0, AngleUnit.DEGREES, 180));
    }

    private boolean isStarted = false;
    private int currentPipeline = DEFAULT_PIPELINE;
    
    // For derivative term in aim assist
    private double previousTxError = 0.0;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        currentResult = getLatestResult();
    }

    public void update() {
        currentResult = getLatestResult();
    }

    /**
     * Start polling for data from the Limelight
     * Must be called before getLatestResult() will return valid data
     */
    public void start() {
        limelight.start();
        isStarted = true;
    }

    /**
     * Stop polling for data from the Limelight
     */
    public void stop() {
        limelight.stop();
        isStarted = false;
    }

    /**
     * Check if the Limelight is currently started
     * @return true if started, false otherwise
     */
    public boolean isStarted() {
        return isStarted;
    }

    /**
     * Switch to a different pipeline
     * @param pipelineIndex Index of the pipeline to switch to (0-9)
     */
    public void switchPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
        currentPipeline = pipelineIndex;
    }

    /**
     * Get the current pipeline index
     * @return Current pipeline index
     */
    public int getCurrentPipeline() {
        return currentPipeline;
    }

    /**
     * Get the latest result from the Limelight
     * @return LLResult object containing target data, or null if not started
     */
    public LLResult getLatestResult() {
        if (!isStarted) {
            return null;
        }
        return limelight.getLatestResult();
    }

    /**
     * Check if a valid AprilTag target is detected
     * @return true if a valid target is detected, false otherwise
     */
    public boolean hasTarget() {
        return currentResult != null && currentResult.isValid();
    }

    /**
     * Check if a specific AprilTag ID is detected
     * @param tagId The AprilTag ID to check for
     * @return true if the specified tag is detected, false otherwise
     */
    public boolean hasAprilTag(int tagId) {
        List<LLResultTypes.FiducialResult> fiducials = getFiducialResults();
        if (fiducials != null) {
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (fiducial.getFiducialId() == tagId) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Get the horizontal offset from the target center (tx)
     * @return Horizontal offset in degrees, or 0.0 if no target
     */
    public double getTx() {
        if (currentResult != null && currentResult.isValid()) {
            return currentResult.getTx();
        }
        return 0.0;
    }

    /**
     * Get the vertical offset from the target center (ty)
     * @return Vertical offset in degrees, or 0.0 if no target
     */
    public double getTy() {
        if (currentResult != null && currentResult.isValid()) {
            return currentResult.getTy();
        }
        return 0.0;
    }

    /**
     * Get the horizontal alignment error for auto alignment
     * This is the same as getTx() but named for clarity
     * @return Horizontal alignment error in degrees (positive = target is to the right)
     */
    public double getAlignmentError() {
        return getTx();
    }

    /**
     * Get the botpose (robot pose) from the Limelight
     * @return Pose3D object containing robot position and orientation, or null if no target
     */
    public Pose3D getBotpose() {
        if (currentResult != null && currentResult.isValid()) {
            return currentResult.getBotpose();
        }
        return null;
    }

    /**
     * Get the distance to the AprilTag in inches
     * Calculates distance from botpose position (x, y, z)
     * @return Distance in inches, or 0.0 if no target
     */
    public double getDistance() {
        Pose3D botpose = getBotpose();
        if (botpose != null) {
            double x = botpose.getPosition().x;
            double y = botpose.getPosition().y;
            double z = botpose.getPosition().z;
            // Calculate 3D distance: sqrt(x^2 + y^2 + z^2)
            return Math.sqrt(x * x + y * y + z * z);
        }
        return 0.0;
    }

    /**
     * Get the horizontal distance to the AprilTag in inches
     * Uses only x and y components (ignoring height)
     * @return Horizontal distance in inches, or 0.0 if no target
     */
    public double getHorizontalDistance() {
        Pose3D botpose = getBotpose();
        if (botpose != null) {
            double x = botpose.getPosition().x;
            double y = botpose.getPosition().y;
            // Calculate 2D horizontal distance: sqrt(x^2 + y^2)
            return Math.sqrt(x * x + y * y);
        }
        return 0.0;
    }

    /**
     * Get fiducial (AprilTag) detection results
     * @return List of FiducialResult objects, or null if no target
     */
    public List<LLResultTypes.FiducialResult> getFiducialResults() {
        if (currentResult != null && currentResult.isValid()) {
            return currentResult.getFiducialResults();
        }
        return null;
    }

    /**
     * Get the first fiducial result (primary AprilTag being tracked)
     * @return First FiducialResult, or null if none found
     */
    public LLResultTypes.FiducialResult getFirstFiducial() {
        List<LLResultTypes.FiducialResult> fiducials = getFiducialResults();
        if (fiducials != null && !fiducials.isEmpty()) {
            return fiducials.get(0);
        }
        return null;
    }

    /**
     * Get the best/closest AprilTag based on target area
     * @return FiducialResult with largest area, or null if none found
     */
    public LLResultTypes.FiducialResult getBestFiducial() {
        List<LLResultTypes.FiducialResult> fiducials = getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return null;
        }

        LLResultTypes.FiducialResult best = fiducials.get(0);
        double maxArea = best.getTargetArea();

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getTargetArea() > maxArea) {
                maxArea = fiducial.getTargetArea();
                best = fiducial;
            }
        }
        return best;
    }

    /**
     * Get a specific AprilTag by ID
     * @param tagId The AprilTag ID to find
     * @return FiducialResult for the specified tag, or null if not found
     */
    public LLResultTypes.FiducialResult getAprilTagById(int tagId) {
        List<LLResultTypes.FiducialResult> fiducials = getFiducialResults();
        if (fiducials != null) {
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (fiducial.getFiducialId() == tagId) {
                    return fiducial;
                }
            }
        }
        return null;
    }

    /**
     * Get the ID of the primary AprilTag being tracked
     * @return AprilTag ID, or -1 if no target
     */
    public int getAprilTagId() {
        LLResultTypes.FiducialResult fiducial = getFirstFiducial();
        if (fiducial != null) {
            return fiducial.getFiducialId();
        }
        return -1;
    }

    /**
     * Get the number of AprilTags currently detected
     * @return Number of detected AprilTags
     */
    public int getAprilTagCount() {
        List<LLResultTypes.FiducialResult> fiducials = getFiducialResults();
        return fiducials != null ? fiducials.size() : 0;
    }
    
    /**
     * Get the target area (ta) from the Limelight
     * @return Target area (0.0 to 1.0), or 0.0 if no target
     */
    public double getTa() {
        if (currentResult != null && currentResult.isValid()) {
            return currentResult.getTa();
        }
        return 0.0;
    }
    
    /**
     * Calculate target shooter velocity based on ty (vertical offset)
     * @param ty Vertical offset from target center in degrees
     * @return Target velocity in ticks per second
     */
    public double calculateTargetVelocity(double ty) {
        double range = MAX_TY - MIN_TY;
        
        // Clamp ty to valid range
        ty = Math.max(MIN_TY, Math.min(MAX_TY, ty));
        
        // Normalize ty to 0-1 range
        double normalized_ty = (ty - MIN_TY) / range;
        
        // Apply power curve
        double norm_ty = Math.pow(normalized_ty, TY_POWER);
        
        // Invert so smaller ty => higher velocity
        double inverted_normalized_ty = 1 - norm_ty;
        inverted_normalized_ty = Math.max(0.0, Math.min(inverted_normalized_ty, 1.0));
        
        // Calculate velocity
        return VELOCITY_OFFSET + (MIN_SHOOTER_VELOCITY +
                (inverted_normalized_ty * (MAX_SHOOTER_VELOCITY - MIN_SHOOTER_VELOCITY)));
    }
    
    /**
     * Get target shooter velocity based on current Limelight reading (using ty)
     * @return Target velocity in ticks per second, or default if no target
     */
    public double getTargetShooterVelocity() {
        if (hasTarget()) {
            return calculateTargetVelocity(getTy());
        }
        return DEFAULT_TARGET_SHOOTER_VELOCITY;
    }
    
    /**
     * Calculate target shooter speed based on Limelight area (ta)
     * Uses piecewise linear interpolation between 6 calibrated points
     * @return Target ticks per second
     */
    public double getTargetShooterTPS() {
        if (!hasTarget()) {
            return DEFAULT_TARGET_SHOOTER_VELOCITY;
        }
        
        double ta = getTa();
        double targetTPS;
        
        if (ta >= LIMELIGHT_AREA_1) {
            targetTPS = SHOOTER_TPS_1;
        } else if (ta >= LIMELIGHT_AREA_2) {
            double areaRange = LIMELIGHT_AREA_1 - LIMELIGHT_AREA_2;
            double speedRange = SHOOTER_TPS_1 - SHOOTER_TPS_2;
            double normalizedArea = (ta - LIMELIGHT_AREA_2) / areaRange;
            targetTPS = SHOOTER_TPS_2 + (normalizedArea * speedRange);
        } else if (ta >= LIMELIGHT_AREA_3) {
            double areaRange = LIMELIGHT_AREA_2 - LIMELIGHT_AREA_3;
            double speedRange = SHOOTER_TPS_2 - SHOOTER_TPS_3;
            double normalizedArea = (ta - LIMELIGHT_AREA_3) / areaRange;
            targetTPS = SHOOTER_TPS_3 + (normalizedArea * speedRange);
        } else if (ta >= LIMELIGHT_AREA_4) {
            double areaRange = LIMELIGHT_AREA_3 - LIMELIGHT_AREA_4;
            double speedRange = SHOOTER_TPS_3 - SHOOTER_TPS_4;
            double normalizedArea = (ta - LIMELIGHT_AREA_4) / areaRange;
            targetTPS = SHOOTER_TPS_4 + (normalizedArea * speedRange);
        } else if (ta >= LIMELIGHT_AREA_5) {
            double areaRange = LIMELIGHT_AREA_4 - LIMELIGHT_AREA_5;
            double speedRange = SHOOTER_TPS_4 - SHOOTER_TPS_5;
            double normalizedArea = (ta - LIMELIGHT_AREA_5) / areaRange;
            targetTPS = SHOOTER_TPS_5 + (normalizedArea * speedRange);
        } else if (ta >= LIMELIGHT_AREA_6) {
            double areaRange = LIMELIGHT_AREA_5 - LIMELIGHT_AREA_6;
            double speedRange = SHOOTER_TPS_5 - SHOOTER_TPS_6;
            double normalizedArea = (ta - LIMELIGHT_AREA_6) / areaRange;
            targetTPS = SHOOTER_TPS_6 + (normalizedArea * speedRange);
        } else {
            targetTPS = SHOOTER_TPS_6;
        }
        
        // Clamp to overall min/max for safety
        if (targetTPS < MIN_SHOOTER_VELOCITY) {
            targetTPS = MIN_SHOOTER_VELOCITY;
        }
        if (targetTPS > MAX_SHOOTER_VELOCITY) {
            targetTPS = MAX_SHOOTER_VELOCITY;
        }
        
        return targetTPS;
    }
    
    /**
     * Calculate aim assist drive power based on tx error with PD control
     * @param targetTx Target tx value to aim for
     * @param isBlue Whether the alliance is blue (unused, kept for compatibility)
     * @return Drive power for turning (-1.0 to 1.0)
     */
    public double calculateAimAssistPower(double targetTx, boolean isBlue) {
        if (!hasTarget()) {
            previousTxError = 0.0; // Reset derivative term when no target
            return 0.0;
        }
        
        double tx = getTx();
        double error = targetTx - tx;
        
        // Calculate derivative term (rate of change of error)
        double derivative = error - previousTxError;
        previousTxError = error; // Store for next iteration
        
        // PDF control: Proportional + Derivative + Feedforward
        double kp = DEFAULT_KP;
        double kd = DEFAULT_KD;
        double kf = DEFAULT_KF;
        
        // Feedforward term: small constant power to help overcome static friction
        double feedforward = Math.signum(error) * kf;
        
        double drivePower = (error * kp) + (derivative * kd) + feedforward;
        
        // Clamp drive power
        if (drivePower > MAX_DRIVE_POWER) {
            drivePower = MAX_DRIVE_POWER;
        }
        if (drivePower < -MAX_DRIVE_POWER) {
            drivePower = -MAX_DRIVE_POWER;
        }
        
        return drivePower;
    }
    
    /**
     * Check if robot is aligned with target based on tx
     * @return true if aligned (within threshold)
     */
    public boolean isAligned() {
        if (!hasTarget()) {
            return false;
        }
        double tx = getTx();
        return tx > -TARGET_TX_THRESHOLD && tx < TARGET_TX_THRESHOLD;
    }
    
    /**
     * Check if robot is aligned for shooting (checks AprilTag alignment with goal offset)
     * Only checks alignment for tags 20 or 24
     * @return true if aligned for shooting
     */
    public boolean isAlignedForShooting() {
        if (!hasTarget()) {
            return false;
        }
        
        int tagId = getAprilTagId();
        if (tagId != 20 && tagId != 24) {
            return false;
        }
        
        // Calculate target offset (goal position behind tag)
        double targetOffset = calculateTargetOffsetForShooting(tagId);
        double currentTx = getTx();
        double error = Math.abs(currentTx - targetOffset);
        
        // Use dynamic tolerance based on distance
        double area = getTa();
        double tolerance = (area >= TARGET_AREA_THRESHOLD) ? 
                          SHOOTER_READY_ALIGNMENT_TOLERANCE_CLOSE : 
                          SHOOTER_READY_ALIGNMENT_TOLERANCE_FAR;
        
        return error < tolerance;
    }
    
    /**
     * Calculate target TX angle to aim at the goal position behind the AprilTag
     * @param tagId The detected AprilTag ID (20 or 24)
     * @return Target tx offset in degrees
     */
    private double calculateTargetOffsetForShooting(int tagId) {
        com.qualcomm.hardware.limelightvision.LLResult result = getLatestResult();
        
        if (result == null || !result.isValid()) {
            return 0.0;
        }
        
        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
            return 0.0;
        }
        
        com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
        org.firstinspires.ftc.robotcore.external.navigation.Pose3D tagPose = fiducial.getTargetPoseCameraSpace();
        
        if (tagPose == null) {
            return 0.0;
        }
        
        double tagX = tagPose.getPosition().x;
        double tagZ = tagPose.getPosition().z;
        double tagYaw = tagPose.getOrientation().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);
        
        // Goal is 508mm (20 inches) behind the tag
        double goalDistanceBehind = 508.0;
        double goalX = tagX + goalDistanceBehind * Math.sin(tagYaw);
        double goalZ = tagZ + goalDistanceBehind * Math.cos(tagYaw);
        
        double targetTx = Math.toDegrees(Math.atan2(goalX, goalZ));
        return targetTx;
    }
    
    /**
     * Get the field position of an AprilTag by ID
     * @param tagId The AprilTag ID
     * @return Pose2D of the tag's field position, or null if not found
     */
    public static Pose2D getAprilTagFieldPosition(int tagId) {
        return APRILTAG_POSITIONS.get(tagId);
    }
    
    /**
     * Calculate the target heading to face an AprilTag (rotation only, no position change)
     * @param aprilTagId The AprilTag ID to target
     * @param currentPose The current robot pose
     * @return Target heading in radians to face the tag, or null if tag position is unknown
     */
    public Double calculateTargetHeading(int aprilTagId, Pose2D currentPose) {
        Pose2D tagPosition = getAprilTagFieldPosition(aprilTagId);
        if (tagPosition == null) {
            return null;
        }
        
        // Get tag position in inches
        double tagX = tagPosition.getX(DistanceUnit.INCH);
        double tagY = tagPosition.getY(DistanceUnit.INCH);
        
        // Get current robot position
        double robotX = currentPose.getX(DistanceUnit.INCH);
        double robotY = currentPose.getY(DistanceUnit.INCH);
        
        // Calculate vector from robot to tag
        double dx = tagX - robotX;
        double dy = tagY - robotY;
        
        // Calculate angle from robot to tag (in field coordinates)
        // In FTC: heading 0 = forward (Y+), heading 90 = right (X+)
        // atan2(dx, dy) gives angle where 0 is forward (Y+), positive is right (X+)
        double angleToTag = Math.atan2(dx, dy);
        
        // Shooter is on the BACK of the robot, so add 180 degrees to face back toward tag
        double angleBackToTag = angleToTag + Math.PI;
        
        // Normalize to [-π, π]
        while (angleBackToTag > Math.PI) angleBackToTag -= 2 * Math.PI;
        while (angleBackToTag < -Math.PI) angleBackToTag += 2 * Math.PI;
        
        return angleBackToTag;
    }
    
    /**
     * Calculate the optimal robot position to see an AprilTag based on current pose
     * This positions the robot at an optimal distance and angle to detect the tag
     * @param aprilTagId The AprilTag ID to target
     * @param currentPose The current robot pose
     * @return Target Pose2D for the robot center, or null if tag position is unknown
     */
    public Pose2D calculateOptimalPosition(int aprilTagId, Pose2D currentPose) {
        Pose2D tagPosition = getAprilTagFieldPosition(aprilTagId);
        if (tagPosition == null) {
            return null;
        }
        
        // Get tag position in inches
        double tagX = tagPosition.getX(DistanceUnit.INCH);
        double tagY = tagPosition.getY(DistanceUnit.INCH);
        
        // Get current robot position
        double robotX = currentPose.getX(DistanceUnit.INCH);
        double robotY = currentPose.getY(DistanceUnit.INCH);
        
        // Calculate vector from robot to tag
        double dx = tagX - robotX;
        double dy = tagY - robotY;
        double distance = Math.sqrt(dx * dx + dy * dy);
        
        // Calculate angle from robot to tag (in field coordinates)
        // In FTC: heading 0 = forward (Y+), heading 90 = right (X+)
        // atan2(dx, dy) gives angle where 0 is forward (Y+), positive is right (X+)
        double angleToTag = Math.atan2(dx, dy);
        
        // Calculate optimal position: move to optimal shooting distance from tag
        // Position robot so it faces the tag
        double optimalDistance = OPTIMAL_SHOOTING_DISTANCE;
        
        // If we're already within range, just adjust position slightly
        if (distance < LIMELIGHT_MAX_RANGE && distance > LIMELIGHT_MIN_RANGE) {
            // Already in range, just ensure we're at optimal distance
            if (Math.abs(distance - optimalDistance) < 6.0) {
                // Close enough, return current position with adjusted heading to face tag
                return new Pose2D(
                        DistanceUnit.INCH,
                        robotX,
                        robotY,
                        AngleUnit.RADIANS,
                        angleToTag
                );
            }
        }
        
        // Calculate target position: optimalDistance away from tag, facing the tag
        // Position robot behind the tag (away from tag) so it faces toward it
        double targetX = tagX - Math.cos(angleToTag) * optimalDistance;
        double targetY = tagY - Math.sin(angleToTag) * optimalDistance;
        
        // Target heading: face the tag (same as angle to tag)
        double targetHeading = angleToTag;
        
        return new Pose2D(
                DistanceUnit.INCH,
                targetX,
                targetY,
                AngleUnit.RADIANS,
                targetHeading
        );
    }
    
    /**
     * Check if the robot is within Limelight detection range of a tag position
     * @param tagId The AprilTag ID
     * @param currentPose The current robot pose
     * @return true if within range, false otherwise
     */
    public boolean isWithinRange(int tagId, Pose2D currentPose) {
        Pose2D tagPosition = getAprilTagFieldPosition(tagId);
        if (tagPosition == null) {
            return false;
        }
        
        double tagX = tagPosition.getX(DistanceUnit.INCH);
        double tagY = tagPosition.getY(DistanceUnit.INCH);
        double robotX = currentPose.getX(DistanceUnit.INCH);
        double robotY = currentPose.getY(DistanceUnit.INCH);
        
        double dx = tagX - robotX;
        double dy = tagY - robotY;
        double distance = Math.sqrt(dx * dx + dy * dy);
        
        return distance >= LIMELIGHT_MIN_RANGE && distance <= LIMELIGHT_MAX_RANGE;
    }
}

