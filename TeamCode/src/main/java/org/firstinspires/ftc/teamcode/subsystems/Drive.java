package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Constants.RobotConstants;

import java.util.List;

/**
 * Drive subsystem - mecanum drive for teleop with odometry
 */
public class Drive {

    private List<LynxModule> allHubs;

    // ========== MECANUM DRIVE (TELEOP) ==========
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private GoBildaPinpointDriver pinpoint;

    // ========== STATE ==========
    private boolean isRedAlliance = true;

    // Cached odometry data (updated only in updateOdometry)
    private double cachedX = 0;  // mm
    private double cachedY = 0;  // mm
    private double cachedHeading = 0;  // degrees
    private double cachedVelocityX = 0;  // mm/sec
    private double cachedVelocityY = 0;  // mm/sec
    private double cachedHeadingVelocity = 0;  // degrees/sec
    private double cachedAccelerationX = 0;  // mm/sec^2
    private double cachedAccelerationY = 0;  // mm/sec^2
    private double cachedHeadingAcceleration = 0;  // degrees/sec^2
    private long lastUpdateTime = 0;

    // Position prediction constants
    private static final double STOPPING_TIME_FULL_SPEED = 0.75;
    private static final double STOPPING_TIME_HALF_SPEED = 0.375;
    private static final double FULL_SPEED_VELOCITY = 2500.0;

    // Active braking constants
    private static final double ACTIVE_BRAKE_INPUT_THRESHOLD = 0.10;
    private static final double ACTIVE_BRAKE_GAIN = 0.0005;

    // Cached drive motor velocities (updated once per loop via cacheDriveVelocities())
    private double cachedFlVelocity = 0;
    private double cachedFrVelocity = 0;
    private double cachedBlVelocity = 0;
    private double cachedBrVelocity = 0;

    // Position override for preset shooting positions
    private boolean positionOverrideActive = false;
    private double overrideX = 0;
    private double overrideY = 0;

    public Drive(HardwareMap hardwareMap) {
        // Enable bulk reading for all hubs
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize drivetrain motors
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
        // Note: Do NOT reset position or recalibrate IMU - allows teleop to continue from where auto left off
        // Match the same constants as Pedro Pathing Constants.java
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-12.5, -55, DistanceUnit.MM);  // strafePodX, forwardPodY (matches Constants.java)
        pinpoint.setEncoderResolution(34.311, DistanceUnit.MM);  // matches Constants.java
        pinpoint.setEncoderDirections(RobotConstants.getEncoderDirectionX(),
                RobotConstants.getEncoderDirectionY());  // matches Constants.java
    }

    /**
     * Re-apply pinpoint settings (matching Pedro Pathing Constants.java).
     */
    public void applyPinpointSettings() {
        pinpoint.setOffsets(-12.5, -55, DistanceUnit.MM);  // strafePodX, forwardPodY
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
    }

    // ========== ALLIANCE ==========

    public void setAlliance(boolean isRed) {
        this.isRedAlliance = isRed;
    }

    public boolean isRedAlliance() {
        return isRedAlliance;
    }

    /**
     * Calculate the turret angle needed to point at the goal, relative to the intake direction.
     * Turret zero position faces the same direction as the intake (robot's front).
     *
     * @param goalX Goal X position in mm (field coordinates)
     * @param goalY Goal Y position in mm (field coordinates)
     * @return Turret angle in degrees (0-360 range, will be clamped to 5-355 in turretAngleToTicks)
     */
    public double calculateTurretAngleToGoal(double goalX, double goalY) {
        // Calculate delta from robot center to goal
        // Note: cachedHeading is in degrees, must convert to radians for Math.cos/sin
        double headingRadians = Math.toRadians(cachedHeading);
        double deltaX = goalX - cachedX - RobotConstants.TURRET_CENTER_OFFSET * Math.cos(headingRadians);
        double deltaY = goalY - cachedY - RobotConstants.TURRET_CENTER_OFFSET * Math.sin(headingRadians);

        // Field angle from robot to goal (0° = positive X axis, increases counter-clockwise)
        double fieldAngleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // Turret angle = angle to goal relative to robot's front (intake direction)
        double turretAngle = fieldAngleToGoal - cachedHeading; // add 90 to convert robot angle to turret angle

        // Normalize to 0-360 range
        while (turretAngle < 0) turretAngle += 360;
        while (turretAngle >= 360) turretAngle -= 360;

        return turretAngle;
    }


    /**
     * Calculate the turret angle to goal for the current alliance.
     * Uses cached odometry position.
     *
     * @param isRedAlliance True if targeting red goal, false for blue
     * @return Turret angle in degrees relative to intake
     */
    public double calculateTurretAngleToGoal(boolean isRedAlliance) {
        double goalX = isRedAlliance ? RobotConstants.GOAL_RED_X : RobotConstants.GOAL_BLUE_X;
        double goalY = isRedAlliance ? RobotConstants.GOAL_RED_Y : RobotConstants.GOAL_BLUE_Y;
        return calculateTurretAngleToGoal(goalX, goalY);
    }

    /**
     * Get the current cached X position in mm.
     */
    public double getCachedX() {
        return cachedX;
    }
    
    /**
     * Get the current cached Y position in mm.
     */
    public double getCachedY() {
        return cachedY;
    }
    
    /**
     * Get the current cached heading in degrees.
     */
    public double getCachedHeading() {
        return cachedHeading;
    }

    // ========== MECANUM DRIVE ==========

    /**
     * Cache all drive motor velocities. Call once per loop at the start,
     * then use getCachedDriveVelocity() and mecanumDriveWithBrakingCached() 
     * to avoid redundant hardware reads.
     */
    public void cacheDriveVelocities() {
        cachedFlVelocity = frontLeft.getVelocity();
        cachedFrVelocity = frontRight.getVelocity();
        cachedBlVelocity = backLeft.getVelocity();
        cachedBrVelocity = backRight.getVelocity();
    }

    /**
     * Get the maximum cached drive motor velocity (ticks/sec).
     * Must call cacheDriveVelocities() first each loop.
     */
    public double getCachedDriveVelocity() {
        double flVel = Math.abs(cachedFlVelocity);
        double frVel = Math.abs(cachedFrVelocity);
        double blVel = Math.abs(cachedBlVelocity);
        double brVel = Math.abs(cachedBrVelocity);
        return Math.max(Math.max(flVel, frVel), Math.max(blVel, brVel));
    }

    /**
     * Calculate heading velocity estimate from cached drive motor velocities.
     * Uses mecanum kinematics: rotation ∝ (right - left) / 4
     * Must call cacheDriveVelocities() first each loop.
     * 
     * @return Heading velocity in motor ticks/sec (positive = counter-clockwise)
     */
    public double getHeadingVelocityFromMotors() {
        double rightSum = cachedFrVelocity + cachedBrVelocity;
        double leftSum = cachedFlVelocity + cachedBlVelocity;
        return (rightSum - leftSum) / 4.0;
    }

    public void mecanumDrive(double drive, double strafe, double turn, double speedMultiplier) {
        // Normal driving
        double d = -drive;
        double s = -strafe;
        double t = turn;

        double frontLeftPower = (d + s + t) * speedMultiplier;
        double frontRightPower = (d - s - t) * speedMultiplier;
        double backLeftPower = (d - s + t) * speedMultiplier;
        double backRightPower = (d + s - t) * speedMultiplier;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public void mecanumDriveWithBraking(double drive, double strafe, double turn, double speedMultiplier) {
        double inputMagnitude = Math.max(Math.abs(drive), Math.max(Math.abs(strafe), Math.abs(turn)));

        // Active braking when input is below threshold - uses cached velocities
        if (inputMagnitude < ACTIVE_BRAKE_INPUT_THRESHOLD) {
            double flPower = Math.max(-1.0, Math.min(1.0, -cachedFlVelocity * ACTIVE_BRAKE_GAIN));
            double frPower = Math.max(-1.0, Math.min(1.0, -cachedFrVelocity * ACTIVE_BRAKE_GAIN));
            double blPower = Math.max(-1.0, Math.min(1.0, -cachedBlVelocity * ACTIVE_BRAKE_GAIN));
            double brPower = Math.max(-1.0, Math.min(1.0, -cachedBrVelocity * ACTIVE_BRAKE_GAIN));

            frontLeft.setPower(flPower);
            frontRight.setPower(frPower);
            backLeft.setPower(blPower);
            backRight.setPower(brPower);
            return;
        }

        // Normal driving
        double d = -drive;
        double s = -strafe;
        double t = turn;

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

    /**
     * Update odometry from Pinpoint hardware. This is the ONLY function that should
     * call Pinpoint hardware methods. All other functions use cached values.
     */
    public void updateOdometry() {
        // Update Pinpoint hardware (single I2C read)
        pinpoint.update();

        // Cache position data
        cachedX = pinpoint.getPosX(DistanceUnit.MM);
        cachedY = pinpoint.getPosY(DistanceUnit.MM);
        cachedHeading = pinpoint.getHeading(AngleUnit.DEGREES);

        // Read new velocity from Pinpoint
        double newVelocityX = pinpoint.getVelX(DistanceUnit.MM);
        double newVelocityY = pinpoint.getVelY(DistanceUnit.MM);
        double newHeadingVelocity = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        // Calculate acceleration from velocity changes (before updating cached velocity)
        long currentTime = System.currentTimeMillis();
        if (lastUpdateTime > 0) {
            double deltaTime = (currentTime - lastUpdateTime) / 1000.0;
            if (deltaTime > 0.005) {
                cachedAccelerationX = (newVelocityX - cachedVelocityX) / deltaTime;
                cachedAccelerationY = (newVelocityY - cachedVelocityY) / deltaTime;
                cachedHeadingAcceleration = (newHeadingVelocity - cachedHeadingVelocity) / deltaTime;
            }
        }

        // Update cached velocity
        cachedVelocityX = newVelocityX;
        cachedVelocityY = newVelocityY;
        cachedHeadingVelocity = newHeadingVelocity;
        lastUpdateTime = currentTime;
    }

    /**
     * Get cached X position (mm). Returns cached value, does not call Pinpoint hardware.
     */
    public double getOdometryX() {
        return cachedX;
    }

    /**
     * Get cached Y position (mm). Returns cached value, does not call Pinpoint hardware.
     */
    public double getOdometryY() {
        return cachedY;
    }

    /**
     * Get cached heading (degrees). Returns cached value, does not call Pinpoint hardware.
     */
    public double getOdometryHeading() {
        return cachedHeading;
    }

    // ========== POSITION PREDICTION ==========

    /**
     * Get cached velocity magnitude (mm/sec). Uses cached velocity values.
     * NOTE: This depends on odometry updates - may be stale if odometry frozen.
     */
    public double getVelocityMagnitude() {
        return Math.sqrt(cachedVelocityX * cachedVelocityX + cachedVelocityY * cachedVelocityY);
    }
    
    /**
     * Get drive motor velocity magnitude from encoder readings.
     * DEPRECATED: Use cacheDriveVelocities() + getCachedDriveVelocity() instead.
     * This method reads directly from motors - expensive hardware call.
     */
    public double getDriveEncoderVelocity() {
        double flVel = Math.abs(frontLeft.getVelocity());
        double frVel = Math.abs(frontRight.getVelocity());
        double blVel = Math.abs(backLeft.getVelocity());
        double brVel = Math.abs(backRight.getVelocity());
        
        // Return max velocity - if any wheel is moving, robot is not stationary
        return Math.max(Math.max(flVel, frVel), Math.max(blVel, brVel));
    }

    /**
     * Calculate stopping time based on current velocity. Uses cached velocity values.
     */
    public double getStoppingTime() {
        double speed = getVelocityMagnitude();
        if (speed < 100) return 0;

        double speedRatio = Math.min(1.0, speed / FULL_SPEED_VELOCITY);
        return STOPPING_TIME_HALF_SPEED + speedRatio * (STOPPING_TIME_FULL_SPEED - STOPPING_TIME_HALF_SPEED);
    }

    /**
     * Get predicted X position. Uses cached position and velocity values.
     */
    public double getPredictedX() {
        if (positionOverrideActive) {
            return overrideX;
        }
        double stoppingTime = getStoppingTime();
        double predictedDistance = cachedVelocityX * stoppingTime / 2.0;
        return cachedX + predictedDistance;
    }

    /**
     * Get predicted Y position. Uses cached position and velocity values.
     */
    public double getPredictedY() {
        if (positionOverrideActive) {
            return overrideY;
        }
        double stoppingTime = getStoppingTime();
        double predictedDistance = cachedVelocityY * stoppingTime / 2.0;
        return cachedY + predictedDistance;
    }

    // ========== POSITION OVERRIDE ==========

    public void setPositionOverride(double xInches, double yInches) {
        positionOverrideActive = true;
        overrideX = xInches * RobotConstants.INCHES_TO_MM;
        overrideY = yInches * RobotConstants.INCHES_TO_MM;
    }

    public void clearPositionOverride() {
        positionOverrideActive = false;
    }

    public boolean isPositionOverrideActive() {
        return positionOverrideActive;
    }

    /**
     * Get cached X velocity (mm/sec). Returns cached value from Pinpoint.
     */
    public double getVelocityX() {
        return cachedVelocityX;
    }

    /**
     * Get cached Y velocity (mm/sec). Returns cached value from Pinpoint.
     */
    public double getVelocityY() {
        return cachedVelocityY;
    }

    /**
     * Get cached heading velocity (degrees/sec). Returns cached value from Pinpoint.
     */
    public double getHeadingVelocity() {
        return cachedHeadingVelocity;
    }

    /**
     * Get cached X acceleration (mm/sec^2). Calculated from velocity changes.
     */
    public double getAccelerationX() {
        return cachedAccelerationX;
    }

    /**
     * Get cached Y acceleration (mm/sec^2). Calculated from velocity changes.
     */
    public double getAccelerationY() {
        return cachedAccelerationY;
    }

    /**
     * Get cached heading acceleration (degrees/sec^2). Calculated from velocity changes.
     */
    public double getHeadingAcceleration() {
        return cachedHeadingAcceleration;
    }

    /**
     * Get cached acceleration magnitude (mm/sec^2).
     */
    public double getAccelerationMagnitude() {
        return Math.sqrt(cachedAccelerationX * cachedAccelerationX + cachedAccelerationY * cachedAccelerationY);
    }

    /**
     * Reset odometry position and IMU. Resets cached values to zero.
     */
    public void resetOdometry() {
        pinpoint.resetPosAndIMU();
        resetCachedValues();
    }
    
    /**
     * Reset all cached odometry values to zero.
     */
    private void resetCachedValues() {
        cachedX = 0;
        cachedY = 0;
        cachedHeading = 0;
        cachedVelocityX = 0;
        cachedVelocityY = 0;
        cachedHeadingVelocity = 0;
        cachedAccelerationX = 0;
        cachedAccelerationY = 0;
        cachedHeadingAcceleration = 0;
        lastUpdateTime = System.currentTimeMillis();
    }

    public void recalibrateIMU() {
        pinpoint.recalibrateIMU();
    }

    /**
     * Set odometry position. Updates Pinpoint hardware and resets cached values.
     */
    public void setOdometryPosition(double fieldX, double fieldY, double fieldHeading) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, fieldX, fieldY, AngleUnit.DEGREES, fieldHeading));
        
        // Reset velocity and acceleration, then set position
        resetCachedValues();
        cachedX = fieldX;
        cachedY = fieldY;
        cachedHeading = fieldHeading;
    }

    public String getPinpointStatus() {
        return pinpoint.getDeviceStatus().toString();
    }

    public double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    public void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}
