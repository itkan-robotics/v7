package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Turret servo control and visual tracking functions
 * Self-contained with its own hardware initialization
 */
public class TurretFunctions {
    
    // ========== HARDWARE ==========
    private Servo turretServo;
    private Servo turretServo2;  // Second turret servo (only used on 19564)
    private Limelight3A limelight;
    
    // Reference to drivetrain for odometry (passed in)
    private DrivetrainFunctions drivetrain;
    
    // ========== CONSTANTS ==========
    // Turret servo positions
    public static final double TURRET_CENTER_POSITION = 0.495;
    public static final double TURRET_90_POSITION = 0.645;
    public static final double TURRET_NEG90_POSITION = 0.335;
    public static final double TURRET_MIN_SERVO = 0.185;//0.3
    public static final double TURRET_MAX_SERVO = 0.85;//0.76
    public static final double TURRET_DEGREES_PER_SERVO_UNIT = 580.65;  // 180° / 0.31 servo units
    public static final double TURRET_SERVO_SPEED = 0.3;  // 0.31 units / 0.8 sec (180° travel time) //0.3875
    // Note: TURRET_FIELD_OFFSET is now in RobotConstants (robot-specific)
    
    // Turret P control constants - use RobotConstants for robot-specific values
    public static final double TURRET_TOLERANCE_FAR = 2.0;
    public static final double TURRET_TOLERANCE_FAR_DIST = 150.0;
    
    
    // Goal positions (mm)
    public static final double GOAL_RED_X_INCHES = 138.0;
    public static final double GOAL_RED_Y_INCHES = 138.0;
    public static final double GOAL_BLUE_X_INCHES = 4.0;
    public static final double GOAL_BLUE_Y_INCHES = 138.0;
    public static final double INCHES_TO_MM = 25.4;
    public static final double GOAL_RED_X = GOAL_RED_X_INCHES * INCHES_TO_MM;
    public static final double GOAL_RED_Y = GOAL_RED_Y_INCHES * INCHES_TO_MM;
    public static final double GOAL_BLUE_X = GOAL_BLUE_X_INCHES * INCHES_TO_MM;
    public static final double GOAL_BLUE_Y = GOAL_BLUE_Y_INCHES * INCHES_TO_MM;
    
    // AprilTag positions
    public static final double APRILTAG_20_X_INCHES = 15.0;
    public static final double APRILTAG_20_Y_INCHES = 128.6;
    public static final double APRILTAG_24_X_INCHES = 127.0;
    public static final double APRILTAG_24_Y_INCHES = 128.6;
    public static final double APRILTAG_20_X = APRILTAG_20_X_INCHES * INCHES_TO_MM;
    public static final double APRILTAG_20_Y = APRILTAG_20_Y_INCHES * INCHES_TO_MM;
    public static final double APRILTAG_24_X = APRILTAG_24_X_INCHES * INCHES_TO_MM;
    public static final double APRILTAG_24_Y = APRILTAG_24_Y_INCHES * INCHES_TO_MM;
    public static final double APRILTAG_20_FACING = 324.0;
    public static final double APRILTAG_24_FACING = 216.0;
    
    // ========== STATE ==========
    private boolean turretUsingVisualTracking = false;
    private double currentTurretKP = 0.0001;  // Will be set dynamically from RobotConstants
    
    private double estimatedServoPosition = TURRET_CENTER_POSITION;
    private double targetServoPosition = TURRET_CENTER_POSITION;
    private long lastServoUpdateTime = System.currentTimeMillis();
    
    private double targetTxOffset = 0.0;
    private double manualTxOffset = 0.0;  // Adjustable offset via dpad
    private double autoTxOffset = 0.0;    // Calculated from field position
    private double farShotTxOffset = 0.0; // Additional offset for far shots
    private boolean closeShotOverride = false; // When true, forces targetTxOffset to 0
    private double currentTurretTolerance = TURRET_TOLERANCE_FAR;
    
    // Cached limelight data (updated once per loop via updateLimelightData())
    private LLResult cachedLimelightResult = null;
    private double cachedTx = 0.0;
    private double cachedTy = 0.0;
    private double cachedTa = 0.0;
    private int cachedTagId = -1;
    private boolean cachedHasTarget = false;
    private double cachedTagDistance = 0.0;
    
    
    public TurretFunctions(HardwareMap hardwareMap, Limelight3A sharedLimelight, DrivetrainFunctions drivetrain) {
        // Initialize turret servo (don't set position until initServos() is called)
        turretServo = hardwareMap.get(Servo.class, "turret_servo");
        
        // Initialize second turret servo if robot has dual turret servos (19564)
        if (RobotConstants.hasDualTurretServos()) {
            turretServo2 = hardwareMap.get(Servo.class, "turret_servo2");
        }
        
        // Use shared Limelight
        this.limelight = sharedLimelight;
        
        // Store reference to drivetrain for odometry
        this.drivetrain = drivetrain;
    }
    
    /**
     * Initialize servo to default position. Call after pinpoint has reset.
     */
    public void initServos() {
        turretServo.setPosition(TURRET_CENTER_POSITION);
        if (RobotConstants.hasDualTurretServos() && turretServo2 != null) {
            turretServo2.setPosition(TURRET_CENTER_POSITION);
        }
        initEstimatedServoPosition(TURRET_CENTER_POSITION);
    }
    
    // ========== SERVO POSITION CONVERSION ==========
    
    public double turretAngleToServoPosition(double turretAngle) {
        // Negative sign to reverse turret direction (matches inverted pinpoint yaw)
        double servoPosition = TURRET_CENTER_POSITION - (turretAngle / TURRET_DEGREES_PER_SERVO_UNIT);
        
        if (servoPosition < TURRET_MIN_SERVO) {
            servoPosition = TURRET_MIN_SERVO;
        } else if (servoPosition > TURRET_MAX_SERVO) {
            servoPosition = TURRET_MAX_SERVO;
        }
        
        return servoPosition;
    }
    
    public double clampTurretServo(double servoPosition) {
        if (servoPosition < TURRET_MIN_SERVO) return TURRET_MIN_SERVO;
        if (servoPosition > TURRET_MAX_SERVO) return TURRET_MAX_SERVO;
        return servoPosition;
    }
    
    // ========== TURRET CONTROL ==========
    
    private void setTurretServos(double position) {
        turretServo.setPosition(position);
        if (RobotConstants.hasDualTurretServos() && turretServo2 != null) {
            turretServo2.setPosition(position);
        }
    }
    
    public void setTurretAngle(double turretAngle) {
        double servoPosition = turretAngleToServoPosition(turretAngle);
        initEstimatedServoPosition(servoPosition);
        setTurretServos(servoPosition);
    }
    
    public void setTurretHome() {
        initEstimatedServoPosition(TURRET_CENTER_POSITION);
        setTurretServos(TURRET_CENTER_POSITION);
    }
    
    public void setTurret90() {
        initEstimatedServoPosition(TURRET_90_POSITION);
        setTurretServos(TURRET_90_POSITION);
    }
    
    public void setTurretNeg90() {
        initEstimatedServoPosition(TURRET_NEG90_POSITION);
        setTurretServos(TURRET_NEG90_POSITION);
    }
    
    public void setTurretMin() {
        initEstimatedServoPosition(TURRET_MIN_SERVO);
        setTurretServos(TURRET_MIN_SERVO);
    }
    
    public void setTurretMax() {
        initEstimatedServoPosition(TURRET_MAX_SERVO);
        setTurretServos(TURRET_MAX_SERVO);
    }
    
    public void setTurretPosition(double position) {
        position = clampTurretServo(position);
        setTurretServos(position);
    }
    
    public double getTurretServoPosition() {
        return turretServo.getPosition();
    }
    
    public double getTurretAngle() {
        double servoPosition = turretServo.getPosition();
        // Negative to match reversed turret direction
        return -(servoPosition - TURRET_CENTER_POSITION) * TURRET_DEGREES_PER_SERVO_UNIT;
    }
    
    // ========== SERVO POSITION ESTIMATION ==========
    
    private void updateEstimatedServoPosition() {
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastServoUpdateTime) / 1000.0;
        lastServoUpdateTime = currentTime;
        
        double maxMovement = TURRET_SERVO_SPEED * deltaTime;
        double positionDiff = targetServoPosition - estimatedServoPosition;
        
        if (Math.abs(positionDiff) <= maxMovement) {
            estimatedServoPosition = targetServoPosition;
        } else {
            estimatedServoPosition += Math.signum(positionDiff) * maxMovement;
        }
    }
    
    public double getEstimatedServoPosition() {
        return estimatedServoPosition;
    }
    
    public void initEstimatedServoPosition(double position) {
        estimatedServoPosition = position;
        targetServoPosition = position;
        lastServoUpdateTime = System.currentTimeMillis();
    }
    
    // ========== GOAL TRACKING ==========
    
    public double calculateTurretAngleToGoal(double goalX, double goalY) {
        // Use predicted position (where robot will stop) instead of current position
        double robotX = drivetrain.getPredictedX();
        double robotY = drivetrain.getPredictedY();
        double robotHeading = drivetrain.getOdometryHeading();
        
        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;
        // Negate to fix turret direction based on field position
        double fieldAngleToGoal = -Math.toDegrees(Math.atan2(deltaY, deltaX));
        
        // Apply robot-specific heading sign (21171 subtracts, 19564 adds due to different yaw scalar)
        double headingSign = RobotConstants.getTurretHeadingSign();
        // Apply robot-specific field offset (19564 needs 180° offset to point at correct goal)
        double fieldOffset = RobotConstants.getTurretFieldOffset();
        double turretAngle = fieldAngleToGoal - (headingSign * robotHeading) - fieldOffset;
        
        while (turretAngle > 180.0) turretAngle -= 360.0;
        while (turretAngle < -180.0) turretAngle += 360.0;
        
        return turretAngle;
    }
    
    public void pointTurretAtGoal(boolean isRedAlliance) {
        // Default to allowing visual tracking
        pointTurretAtGoal(isRedAlliance, true);
    }
    
    public void pointTurretAtGoal(boolean isRedAlliance, boolean allowVisualTracking) {
        int expectedTagId = isRedAlliance ? 24 : 20;
        int detectedTagId = getDetectedAprilTagId(isRedAlliance);
        
        // Only use visual tracking if allowed AND tag is detected
        if (allowVisualTracking && detectedTagId == expectedTagId) {
            turretUsingVisualTracking = true;
            pointTurretVisual(isRedAlliance);
        } else {
            turretUsingVisualTracking = false;
            pointTurretByPosition(isRedAlliance);
        }
    }
    
    private void pointTurretVisual(boolean isRedAlliance) {
        updateEstimatedServoPosition();
        
        double tx = getLimelightTx(isRedAlliance);
        
        // Close shot override: force target TX to 0
        if (closeShotOverride) {
            targetTxOffset = 0.0;
        } else {
            // Calculate auto offset from field position + manual adjustment + far shot offset
            // Negate offset for blue alliance (calibrated for red)
            calculateAutoTxOffset();
            double allianceAutoOffset = isRedAlliance ? autoTxOffset : -autoTxOffset;
            targetTxOffset = allianceAutoOffset + manualTxOffset + farShotTxOffset;
        }
        
        // Get distance for dynamic KP
        double ta = getAprilTagArea();
        double distanceInches = 100.0;  // Default distance estimate
        if (ta > 0.1) {
            // Rough distance estimate from area (requires meaningful tag size)
            distanceInches = 50.0 / Math.sqrt(ta);
        }
        
        currentTurretKP = calculateDynamicKP(distanceInches);
        
        double error = targetTxOffset - tx;
        double adjustment = currentTurretKP * error;
        
        double newServo = estimatedServoPosition + adjustment;
        newServo = clampTurretServo(newServo);
        
        setTurretServos(newServo);
        targetServoPosition = newServo;
    }
    
    private void pointTurretByPosition(boolean isRedAlliance) {
        updateEstimatedServoPosition();
        
        double goalX = isRedAlliance ? GOAL_RED_X : GOAL_BLUE_X;
        double goalY = isRedAlliance ? GOAL_RED_Y : GOAL_BLUE_Y;
        
        double turretAngle = calculateTurretAngleToGoal(goalX, goalY);
        double servoPosition = turretAngleToServoPosition(turretAngle);
        servoPosition = clampTurretServo(servoPosition);
        
        // Position mode: set servo directly without P control
        setTurretServos(servoPosition);
        targetServoPosition = servoPosition;
    }
    
    private double calculateDynamicKP(double distanceInches) {
        double kpFar = RobotConstants.getTurretKpFar();
        double kpClose = RobotConstants.getTurretKpClose();
        double closeDist = RobotConstants.getTurretToleranceCloseDist();
        
        if (distanceInches >= TURRET_TOLERANCE_FAR_DIST) return kpFar;
        if (distanceInches <= closeDist) return kpClose;
        
        double range = TURRET_TOLERANCE_FAR_DIST - closeDist;
        double t = (distanceInches - closeDist) / range;
        return kpClose + (kpFar - kpClose) * t;
    }
    
    private double calculateDynamicTolerance(double distanceInches) {
        double toleranceClose = RobotConstants.getTurretToleranceClose();
        double closeDist = RobotConstants.getTurretToleranceCloseDist();
        
        if (distanceInches >= TURRET_TOLERANCE_FAR_DIST) return TURRET_TOLERANCE_FAR;
        if (distanceInches <= closeDist) return toleranceClose;
        
        double range = TURRET_TOLERANCE_FAR_DIST - closeDist;
        double t = (distanceInches - closeDist) / range;
        return toleranceClose + (TURRET_TOLERANCE_FAR - toleranceClose) * t;
    }
    
    // ========== STATE GETTERS ==========
    
    public double getCurrentTurretKP() {
        return currentTurretKP;
    }
    
    public double getTargetTxOffset() {
        return targetTxOffset;
    }
    
    public double getManualTxOffset() {
        return manualTxOffset;
    }
    
    public void setManualTxOffset(double offset) {
        manualTxOffset = offset;
    }
    
    public void adjustManualTxOffset(double delta) {
        manualTxOffset += delta;
    }
    
    public void setFarShotTxOffset(double offset) {
        farShotTxOffset = offset;
    }
    
    public void clearFarShotTxOffset() {
        farShotTxOffset = 0.0;
    }
    
    public double getFarShotTxOffset() {
        return farShotTxOffset;
    }
    
    /**
     * Enable close shot override - forces target TX to 0
     */
    public void setCloseShotOverride(boolean enabled) {
        closeShotOverride = enabled;
    }
    
    public boolean isCloseShotOverride() {
        return closeShotOverride;
    }
    
    /**
     * Calculate tx offset based on field position
     * Data points:
     * FX:89, FY:144 → tx: 8.4
     * FX:64, FY:144 → tx: 9.4
     * FX:26, FY:146 → tx: 0
     * FX:70, FY:84 → tx: 0
     * FX:94, FY:99 → tx: 0
     * FX:66, FY:35 → tx: -4
     */
    public double calculateAutoTxOffset() {
        // Use predicted position for tx offset calculation
        double fieldX = drivetrain.getPredictedX() / INCHES_TO_MM;
        double fieldY = drivetrain.getPredictedY() / INCHES_TO_MM;
        
        // High Y (close to goal baseline ~144) - positive offset needed
        // Low Y (far from goal) - negative or zero offset
        
        double txOffset = 0.0;
        
        if (fieldY >= 130) {
            // Near goal baseline (Y ~144-146)
            // Offset varies by X: around 8-9 for center, 0 for edges
            if (fieldX >= 50 && fieldX <= 100) {
                // Center area - interpolate between data points
                // FX:64 → 9.4, FX:89 → 8.4
                double t = (fieldX - 64) / (89 - 64);
                txOffset = 9.4 + t * (8.4 - 9.4);  // 9.4 to 8.4
            } else if (fieldX < 50) {
                // Left side - fade to 0
                double t = fieldX / 50.0;
                txOffset = t * 9.4;  // 0 to 9.4
            } else {
                // Right side (X > 100) - fade to 0
                double t = (fieldX - 100) / 30.0;
                t = Math.min(1.0, t);
                txOffset = 8.4 * (1.0 - t);  // 8.4 to 0
            }
        } else if (fieldY >= 80) {
            // Mid-field (Y ~84-99) - offset is 0
            txOffset = 0.0;
        } else if (fieldY >= 30) {
            // Far from goal (Y ~35-80) - negative offset
            // Interpolate from 0 at Y=80 to -4 at Y=35
            double t = (80 - fieldY) / (80 - 35);
            txOffset = -2.0 * t;
        } else {
            // Very far - use max negative
            txOffset = -2.0;
        }
        
        autoTxOffset = txOffset;
        return txOffset;
    }
    
    public double getAutoTxOffset() {
        return autoTxOffset;
    }
    
    public boolean isTurretUsingVisualTracking() {
        return turretUsingVisualTracking;
    }
    
    public boolean isTurretOnTarget(boolean isRedAlliance) {
        if (!turretUsingVisualTracking) return false;
        
        double distanceToTag = getAprilTagDistance(isRedAlliance) / 25.4;
        currentTurretTolerance = calculateDynamicTolerance(distanceToTag);
        
        double tx = getLimelightTx(isRedAlliance);
        double error = Math.abs(tx - targetTxOffset);
        return error < currentTurretTolerance;
    }
    
    public double getCurrentTurretTolerance() {
        return currentTurretTolerance;
    }
    
    public double getAngleToGoal(boolean isRedAlliance) {
        double goalX = isRedAlliance ? GOAL_RED_X : GOAL_BLUE_X;
        double goalY = isRedAlliance ? GOAL_RED_Y : GOAL_BLUE_Y;
        
        double robotX = drivetrain.getOdometryX();
        double robotY = drivetrain.getOdometryY();
        
        return Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));
    }
    
    public double getDistanceToGoal(boolean isRedAlliance) {
        double goalX = isRedAlliance ? GOAL_RED_X : GOAL_BLUE_X;
        double goalY = isRedAlliance ? GOAL_RED_Y : GOAL_BLUE_Y;
        
        double robotX = drivetrain.getOdometryX();
        double robotY = drivetrain.getOdometryY();
        
        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;
        
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }
    
    
    // ========== LIMELIGHT FUNCTIONS ==========
    
    /**
     * Update all cached limelight data. Call this ONCE at the start of each loop.
     * All getter methods will use the cached data for consistency and performance.
     */
    public void updateLimelightData(boolean isRedAlliance) {
        cachedLimelightResult = limelight.getLatestResult();
        
        // Reset cached values
        cachedTx = 0.0;
        cachedTy = 0.0;
        cachedTa = 0.0;
        cachedTagId = -1;
        cachedHasTarget = false;
        cachedTagDistance = 0.0;
        
        if (cachedLimelightResult != null && cachedLimelightResult.isValid()) {
            cachedHasTarget = true;
            cachedTx = cachedLimelightResult.getTx();
            cachedTy = cachedLimelightResult.getTy();
            cachedTa = cachedLimelightResult.getTa();
            
            // Find the goal tag for this alliance
            int targetTagId = isRedAlliance ? 24 : 20;
            LLResultTypes.FiducialResult goalTag = findTargetTagCached(targetTagId);
            
            if (goalTag != null) {
                cachedTagId = (int) goalTag.getFiducialId();
                
                // Calculate distance from tag pose
                org.firstinspires.ftc.robotcore.external.navigation.Pose3D cameraPose = goalTag.getTargetPoseCameraSpace();
                if (cameraPose != null) {
                    double x = cameraPose.getPosition().x;
                    double y = cameraPose.getPosition().y;
                    double z = cameraPose.getPosition().z;
                    cachedTagDistance = Math.sqrt(x * x + y * y + z * z);
                }
            }
        }
    }
    
    private LLResultTypes.FiducialResult findTargetTagCached(int targetTagId) {
        if (cachedLimelightResult != null && cachedLimelightResult.isValid()) {
            if (cachedLimelightResult.getFiducialResults() != null && !cachedLimelightResult.getFiducialResults().isEmpty()) {
                for (LLResultTypes.FiducialResult fiducial : cachedLimelightResult.getFiducialResults()) {
                    if ((int) fiducial.getFiducialId() == targetTagId) {
                        return fiducial;
                    }
                }
            }
        }
        return null;
    }
    
    // Cached getters - use these after calling updateLimelightData()
    public int getDetectedAprilTagId(boolean isRedAlliance) {
        return cachedTagId;
    }
    
    public double getAprilTagArea() {
        return cachedTa;
    }
    
    public double getAprilTagDistance(boolean isRedAlliance) {
        return cachedTagDistance;
    }
    
    public double getLimelightTx(boolean isRedAlliance) {
        // Only return tx if we have the correct tag
        if (cachedTagId > 0) {
            return cachedTx;
        }
        return 0.0;
    }
    
    public double getLimelightTy() {
        return cachedTy;
    }
    
    public boolean hasLimelightTarget() {
        return cachedHasTarget;
    }
    
    public boolean isCorrectAprilTagVisible(boolean isRedAlliance) {
        int expectedTagId = isRedAlliance ? 24 : 20;
        return cachedTagId == expectedTagId;
    }
    
    public double getAprilTagYaw() {
        if (cachedLimelightResult != null && cachedLimelightResult.isValid()) {
            if (cachedLimelightResult.getFiducialResults() != null && !cachedLimelightResult.getFiducialResults().isEmpty()) {
                LLResultTypes.FiducialResult fiducial = cachedLimelightResult.getFiducialResults().get(0);
                org.firstinspires.ftc.robotcore.external.navigation.Pose3D robotPose = fiducial.getRobotPoseTargetSpace();
                if (robotPose != null) {
                    return robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                }
            }
        }
        return 0.0;
    }
    
    /**
     * Get the cached LLResult for use by other systems (like ShooterFunctions)
     */
    public LLResult getCachedLimelightResult() {
        return cachedLimelightResult;
    }
    
    // ========== POSITION RESET FROM APRILTAG ==========
    
    public boolean resetPositionFromAprilTag(boolean isRedAlliance, com.qualcomm.hardware.gobilda.GoBildaPinpointDriver pinpoint) {
        int expectedTagId = isRedAlliance ? 24 : 20;
        int detectedTagId = getDetectedAprilTagId(isRedAlliance);
        
        if (detectedTagId != expectedTagId) return false;
        
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return false;
        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) return false;
        
        LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
        org.firstinspires.ftc.robotcore.external.navigation.Pose3D tagPoseCameraSpace = fiducial.getTargetPoseCameraSpace();
        
        if (tagPoseCameraSpace == null) return false;
        
        double tagX_cam = tagPoseCameraSpace.getPosition().x * 1000.0;
        double tagZ_cam = tagPoseCameraSpace.getPosition().z * 1000.0;
        double tagYaw_cam = tagPoseCameraSpace.getOrientation().getYaw(AngleUnit.DEGREES);
        
        double tagFieldX, tagFieldY, tagFieldFacing;
        
        if (expectedTagId == 20) {
            tagFieldX = APRILTAG_20_X;
            tagFieldY = APRILTAG_20_Y;
            tagFieldFacing = APRILTAG_20_FACING;
        } else {
            tagFieldX = APRILTAG_24_X;
            tagFieldY = APRILTAG_24_Y;
            tagFieldFacing = APRILTAG_24_FACING;
        }
        
        double fieldHeading = tagFieldFacing + 180.0 - tagYaw_cam;
        while (fieldHeading > 180.0) fieldHeading -= 360.0;
        while (fieldHeading < -180.0) fieldHeading += 360.0;
        
        double headingRad = Math.toRadians(fieldHeading);
        
        double fieldRobotX = tagFieldX - (tagZ_cam * Math.cos(headingRad) - tagX_cam * Math.sin(headingRad));
        double fieldRobotY = tagFieldY - (tagZ_cam * Math.sin(headingRad) + tagX_cam * Math.cos(headingRad));
        
        double currentFieldHeading = drivetrain.getOdometryHeading();
        
        double pinpointX = drivetrain.getRobotStartY() - fieldRobotY;
        double pinpointY = fieldRobotX - drivetrain.getRobotStartX();
        double pinpointHeading = currentFieldHeading - drivetrain.getRobotStartHeading();
        
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, pinpointX, pinpointY, AngleUnit.DEGREES, pinpointHeading));
        
        return true;
    }
}
