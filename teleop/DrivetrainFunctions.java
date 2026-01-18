package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Drivetrain functions including mecanum drive, odometry, and autonomous movement
 * Self-contained with its own hardware initialization
 */
public class DrivetrainFunctions {
    
    // ========== HARDWARE ==========
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private GoBildaPinpointDriver pinpoint;
    
    // ========== CONSTANTS ==========
    // Autonomous drive constants
    public static final double COUNTS_PER_MOTOR_REV = 537.7;
    public static final double WHEEL_DIAMETER_MM = 96.0;
    public static final double COUNTS_PER_MM = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_MM);
    public static final double DRIVE_HEADING_TOLERANCE = 2.0;
    
    // Robot starting positions (inches) - same for both alliances
    public static final double ROBOT_START_X_RED_INCHES = 95.0;
    public static final double ROBOT_START_Y_RED_INCHES = 70.0;
    public static final double ROBOT_START_X_BLUE_INCHES = 50.0;
    public static final double ROBOT_START_Y_BLUE_INCHES = 70.0;
    public static final double ROBOT_START_HEADING_RED = 270.0 - 45;
    public static final double ROBOT_START_HEADING_BLUE = 270.0 + 45;

    
    // Field center position (for position reset)
    public static final double FIELD_CENTER_X_INCHES = 71.7;
    public static final double FIELD_CENTER_Y_INCHES = 68.0;
    
    // Conversion
    public static final double INCHES_TO_MM = 25.4;
    public static final double ROBOT_START_X_RED = ROBOT_START_X_RED_INCHES * INCHES_TO_MM;
    public static final double ROBOT_START_Y_RED = ROBOT_START_Y_RED_INCHES * INCHES_TO_MM;
    public static final double ROBOT_START_X_BLUE = ROBOT_START_X_BLUE_INCHES * INCHES_TO_MM;
    public static final double ROBOT_START_Y_BLUE = ROBOT_START_Y_BLUE_INCHES * INCHES_TO_MM;
    
    // ========== STATE ==========
    private boolean isRedAlliance = true;
    
    // Velocity tracking for position prediction
    private double lastOdoX = 0;
    private double lastOdoY = 0;
    private long lastVelocityTime = 0;
    private double velocityX = 0;  // mm/sec
    private double velocityY = 0;  // mm/sec
    private static final double STOPPING_TIME_FULL_SPEED = 0.75;  // seconds to stop from full speed
    private static final double STOPPING_TIME_HALF_SPEED = 0.375;  // seconds to stop from half speed
    private static final double FULL_SPEED_VELOCITY = 2500.0;    // estimated mm/sec at full speed
    
    // Active braking constants
    private static final double ACTIVE_BRAKE_INPUT_THRESHOLD = 0.10;  // Below 10% input triggers active brake
    private static final double ACTIVE_BRAKE_GAIN = 0.0005;  // Power per tick/sec of motor velocity
    
    // Position override for preset shooting positions
    private boolean positionOverrideActive = false;
    private double overrideX = 0;  // mm
    private double overrideY = 0;  // mm
    
    public DrivetrainFunctions(HardwareMap hardwareMap) {
        // Initialize drivetrain motors (DcMotorEx for velocity reading)
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");

        // Set motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set brake mode
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize Pinpoint Odometry
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(RobotConstants.getPinpointOffsetX(), 
                            RobotConstants.getPinpointOffsetY(), DistanceUnit.MM);
        pinpoint.setEncoderResolution(34.31, DistanceUnit.MM);
        pinpoint.setEncoderDirections(RobotConstants.getEncoderDirectionX(), 
                                       RobotConstants.getEncoderDirectionY());
        pinpoint.setYawScalar(RobotConstants.getYawScalar());
        // Note: resetPosAndIMU() is called from MainTeleOp after servos settle
    }
    
    /**
     * Re-apply pinpoint settings based on current RobotConstants.
     * Call this after changing the robot selection to update encoder directions, yaw scalar, and offsets.
     */
    public void applyPinpointSettings() {
        pinpoint.setOffsets(RobotConstants.getPinpointOffsetX(), 
                            RobotConstants.getPinpointOffsetY(), DistanceUnit.MM);
        pinpoint.setEncoderDirections(RobotConstants.getEncoderDirectionX(), 
                                       RobotConstants.getEncoderDirectionY());
        pinpoint.setYawScalar(RobotConstants.getYawScalar());
    }
    
    // ========== ALLIANCE ==========
    
    public void setAlliance(boolean isRed) {
        this.isRedAlliance = isRed;
    }
    
    public boolean isRedAlliance() {
        return isRedAlliance;
    }
    
    public double getRobotStartX() {
        return isRedAlliance ? ROBOT_START_X_RED : ROBOT_START_X_BLUE;
    }
    
    public double getRobotStartY() {
        return isRedAlliance ? ROBOT_START_Y_RED : ROBOT_START_Y_BLUE;
    }
    public double getRobotStartHeading() {
        return isRedAlliance ? ROBOT_START_HEADING_RED : ROBOT_START_HEADING_BLUE;
    }
    
    // ========== MECANUM DRIVE ==========
    
    public void mecanumDrive(double drive, double strafe, double turn, double speedMultiplier) {
        // Calculate input magnitude to check for active braking
        double inputMagnitude = Math.max(Math.abs(drive), Math.max(Math.abs(strafe), Math.abs(turn)));
        
        // Active braking: when input is below threshold, actively brake to stop
        if (inputMagnitude < ACTIVE_BRAKE_INPUT_THRESHOLD) {
            // Use motor encoder velocities directly for each wheel
            double flVel = frontLeft.getVelocity();
            double frVel = frontRight.getVelocity();
            double blVel = backLeft.getVelocity();
            double brVel = backRight.getVelocity();
            
            // Apply opposing power to each motor based on its velocity
            // Negative velocity * positive gain = positive power to slow down
            double flPower = -flVel * ACTIVE_BRAKE_GAIN;
            double frPower = -frVel * ACTIVE_BRAKE_GAIN;
            double blPower = -blVel * ACTIVE_BRAKE_GAIN;
            double brPower = -brVel * ACTIVE_BRAKE_GAIN;
            
            // Clamp powers
            flPower = Math.max(-1.0, Math.min(1.0, flPower));
            frPower = Math.max(-1.0, Math.min(1.0, frPower));
            blPower = Math.max(-1.0, Math.min(1.0, blPower));
            brPower = Math.max(-1.0, Math.min(1.0, brPower));
            
            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);
            return;
        }
        
        // Normal driving
        // Negate drive and strafe (but not turn)
        double d = -drive;
        double s = -strafe;
        double t = turn;  // Turn is not negated
        
        double frontLeftPower = (d + s + t) * speedMultiplier;
        double frontRightPower = (d - s - t) * speedMultiplier;
        double backLeftPower = (d - s + t) * speedMultiplier;
        double backRightPower = (d + s - t) * speedMultiplier;
        
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }
    
    public void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    
    // ========== ODOMETRY ==========
    
    public void updateOdometry() {
        pinpoint.update();
        
        // Calculate velocity for position prediction
        long currentTime = System.currentTimeMillis();
        double currentX = getOdometryX();
        double currentY = getOdometryY();
        
        if (lastVelocityTime > 0) {
            double deltaTime = (currentTime - lastVelocityTime) / 1000.0;  // seconds
            if (deltaTime > 0.005) {  // Minimum 5ms
                velocityX = (currentX - lastOdoX) / deltaTime;
                velocityY = (currentY - lastOdoY) / deltaTime;
            }
        }
        
        lastOdoX = currentX;
        lastOdoY = currentY;
        lastVelocityTime = currentTime;
    }
    
    public double getOdometryX() {
        double pinpointY = pinpoint.getPosY(DistanceUnit.MM);
        return getRobotStartX() + pinpointY;
    }
    
    public double getOdometryY() {
        double pinpointX = pinpoint.getPosX(DistanceUnit.MM);
        return getRobotStartY() + pinpointX;
    }
    
    public double getOdometryHeading() {
        double pinpointHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        double fieldHeading = pinpointHeading + getRobotStartHeading();
        
        while (fieldHeading >= 360.0) fieldHeading -= 360.0;
        while (fieldHeading < 0.0) fieldHeading += 360.0;
        
        return fieldHeading;
    }
    
    public double getRawIMUHeading() {
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }
    
    // ========== POSITION PREDICTION ==========
    
    /**
     * Get the current velocity magnitude in mm/sec
     */
    public double getVelocityMagnitude() {
        return Math.sqrt(velocityX * velocityX + velocityY * velocityY);
    }
    
    /**
     * Calculate stopping time based on current velocity
     * Full speed (~1500mm/s) = 2 sec, half speed = 1 sec, linear interpolation
     */
    public double getStoppingTime() {
        double speed = getVelocityMagnitude();
        if (speed < 100) return 0;  // Already nearly stopped
        
        // Linear interpolation: faster = longer stopping time
        double speedRatio = Math.min(1.0, speed / FULL_SPEED_VELOCITY);
        return STOPPING_TIME_HALF_SPEED + speedRatio * (STOPPING_TIME_FULL_SPEED - STOPPING_TIME_HALF_SPEED);
    }
    
    /**
     * Get predicted X position where robot will stop
     * Assumes constant deceleration (velocity decreases linearly to 0)
     * Distance = velocity * stoppingTime / 2 (triangle area)
     * Returns override position if override is active
     */
    public double getPredictedX() {
        if (positionOverrideActive) {
            return overrideX;
        }
        double stoppingTime = getStoppingTime();
        double predictedDistance = velocityX * stoppingTime / 2.0;
        return getOdometryX() + predictedDistance;
    }
    
    /**
     * Get predicted Y position where robot will stop
     * Returns override position if override is active
     */
    public double getPredictedY() {
        if (positionOverrideActive) {
            return overrideY;
        }
        double stoppingTime = getStoppingTime();
        double predictedDistance = velocityY * stoppingTime / 2.0;
        return getOdometryY() + predictedDistance;
    }
    
    // ========== POSITION OVERRIDE ==========
    
    /**
     * Set a temporary position override (used for preset shooting positions)
     * @param xInches X position in inches
     * @param yInches Y position in inches
     */
    public void setPositionOverride(double xInches, double yInches) {
        positionOverrideActive = true;
        overrideX = xInches * INCHES_TO_MM;
        overrideY = yInches * INCHES_TO_MM;
    }
    
    /**
     * Clear the position override and return to odometry-based position
     */
    public void clearPositionOverride() {
        positionOverrideActive = false;
    }
    
    /**
     * Check if position override is currently active
     */
    public boolean isPositionOverrideActive() {
        return positionOverrideActive;
    }
    
    public double getVelocityX() {
        return velocityX;
    }
    
    public double getVelocityY() {
        return velocityY;
    }
    
    public double getPinpointHeading() {
        double pinpointHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        double fieldHeading = pinpointHeading + getRobotStartHeading();
        
        while (fieldHeading >= 360.0) fieldHeading -= 360.0;
        while (fieldHeading < 0.0) fieldHeading += 360.0;
        
        return fieldHeading;
    }
    
    public double getRawPinpointX() {
        return pinpoint.getPosX(DistanceUnit.MM);
    }
    
    public double getRawPinpointY() {
        return pinpoint.getPosY(DistanceUnit.MM);
    }
    
    public void resetOdometry() {
        pinpoint.resetPosAndIMU();
    }
    
    public void recalibrateIMU() {
        pinpoint.recalibrateIMU();
    }
    
    public void resetPosition() {
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 
            pinpoint.getHeading(AngleUnit.DEGREES)));
    }
    
    public void setOdometryPosition(double fieldX, double fieldY, double fieldHeading) {
        double pinpointX = getRobotStartY() - fieldY;
        double pinpointY = fieldX - getRobotStartX();
        double pinpointHeading = fieldHeading - getRobotStartHeading();
        
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, pinpointX, pinpointY, AngleUnit.DEGREES, pinpointHeading));
    }
    
    public void resetPositionToFieldCenter() {
        double currentHeading = getOdometryHeading();
        double fieldCenterX = FIELD_CENTER_X_INCHES * INCHES_TO_MM;
        double fieldCenterY = FIELD_CENTER_Y_INCHES * INCHES_TO_MM;
        setOdometryPosition(fieldCenterX, fieldCenterY, currentHeading);
    }
    
    public String getPinpointStatus() {
        return pinpoint.getDeviceStatus().toString();
    }
    
    public double getPinpointYawScalar() {
        return pinpoint.getYawScalar();
    }
    
    public double getPinpointFrequency() {
        return pinpoint.getFrequency();
    }
    
    // ========== ENCODER-BASED AUTONOMOUS ==========
    
    public void driveDistance(double distanceMM, double power) {
        int targetCounts = (int)(distanceMM * COUNTS_PER_MM);
        
        int flTarget = frontLeft.getCurrentPosition() + targetCounts;
        int frTarget = frontRight.getCurrentPosition() + targetCounts;
        int blTarget = backLeft.getCurrentPosition() + targetCounts;
        int brTarget = backRight.getCurrentPosition() + targetCounts;
        
        frontLeft.setTargetPosition(flTarget);
        frontRight.setTargetPosition(frTarget);
        backLeft.setTargetPosition(blTarget);
        backRight.setTargetPosition(brTarget);
        
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        frontLeft.setPower(Math.abs(power));
        frontRight.setPower(Math.abs(power));
        backLeft.setPower(Math.abs(power));
        backRight.setPower(Math.abs(power));
    }
    
    public void strafeDistance(double distanceMM, double power) {
        int targetCounts = (int)(distanceMM * COUNTS_PER_MM);
        
        int flTarget = frontLeft.getCurrentPosition() + targetCounts;
        int frTarget = frontRight.getCurrentPosition() - targetCounts;
        int blTarget = backLeft.getCurrentPosition() - targetCounts;
        int brTarget = backRight.getCurrentPosition() + targetCounts;
        
        frontLeft.setTargetPosition(flTarget);
        frontRight.setTargetPosition(frTarget);
        backLeft.setTargetPosition(blTarget);
        backRight.setTargetPosition(brTarget);
        
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        frontLeft.setPower(Math.abs(power));
        frontRight.setPower(Math.abs(power));
        backLeft.setPower(Math.abs(power));
        backRight.setPower(Math.abs(power));
    }
    
    public boolean rotateToHeading(double targetHeading, double power) {
        double currentHeading = getOdometryHeading();
        double errorHeading = normalizeAngle(targetHeading - currentHeading);
        
        if (Math.abs(errorHeading) < DRIVE_HEADING_TOLERANCE) {
            mecanumDrive(0, 0, 0, 1.0);
            return true;
        }
        
        double turnPower = (errorHeading > 0) ? Math.abs(power) : -Math.abs(power);
        mecanumDrive(0, 0, turnPower, 1.0);
        
        return false;
    }
    
    public boolean isDriveBusy() {
        return frontLeft.isBusy() || frontRight.isBusy() || 
               backLeft.isBusy() || backRight.isBusy();
    }
    
    public void stopDriveAndReset() {
        stopMotors();
        
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    public double getHeadingError(double targetHeading) {
        double currentHeading = getOdometryHeading();
        return normalizeAngle(targetHeading - currentHeading);
    }
    
    public double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }
    
    // ========== MOTOR ACCESS (for testing) ==========
    
    public int getFrontLeftPosition() {
        return frontLeft.getCurrentPosition();
    }
    
    public int getFrontRightPosition() {
        return frontRight.getCurrentPosition();
    }
    
    public int getBackLeftPosition() {
        return backLeft.getCurrentPosition();
    }
    
    public int getBackRightPosition() {
        return backRight.getCurrentPosition();
    }
}
