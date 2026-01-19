package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
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

    // ========== CONSTANTS ==========
    public static final double COUNTS_PER_MOTOR_REV = 537.7;
    public static final double WHEEL_DIAMETER_MM = 96.0;
    public static final double COUNTS_PER_MM = COUNTS_PER_MOTOR_REV / (Math.PI * WHEEL_DIAMETER_MM);
    public static final double DRIVE_HEADING_TOLERANCE = 2.0;

    // Robot starting positions (inches)


    // Field center position (for position reset)
    public static final double FIELD_CENTER_X_INCHES = 71.7;
    public static final double FIELD_CENTER_Y_INCHES = 68.0;

    // Conversion
    public static final double INCHES_TO_MM = 25.4;

    // ========== STATE ==========
    private boolean isRedAlliance = true;

    // Velocity tracking for position prediction
    private double lastOdoX = 0;
    private double lastOdoY = 0;
    private long lastVelocityTime = 0;
    private double velocityX = 0;  // mm/sec
    private double velocityY = 0;  // mm/sec
    private static final double STOPPING_TIME_FULL_SPEED = 0.75;
    private static final double STOPPING_TIME_HALF_SPEED = 0.375;
    private static final double FULL_SPEED_VELOCITY = 2500.0;

    // Active braking constants
    private static final double ACTIVE_BRAKE_INPUT_THRESHOLD = 0.10;
    private static final double ACTIVE_BRAKE_GAIN = 0.0005;

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
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);  // matches Constants.java
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

    // ========== MECANUM DRIVE ==========

    public void mecanumDrive(double drive, double strafe, double turn, double speedMultiplier) {
        double inputMagnitude = Math.max(Math.abs(drive), Math.max(Math.abs(strafe), Math.abs(turn)));

        // Active braking when input is below threshold
        if (inputMagnitude < ACTIVE_BRAKE_INPUT_THRESHOLD) {
            double flVel = frontLeft.getVelocity();
            double frVel = frontRight.getVelocity();
            double blVel = backLeft.getVelocity();
            double brVel = backRight.getVelocity();

            double flPower = Math.max(-1.0, Math.min(1.0, -flVel * ACTIVE_BRAKE_GAIN));
            double frPower = Math.max(-1.0, Math.min(1.0, -frVel * ACTIVE_BRAKE_GAIN));
            double blPower = Math.max(-1.0, Math.min(1.0, -blVel * ACTIVE_BRAKE_GAIN));
            double brPower = Math.max(-1.0, Math.min(1.0, -brVel * ACTIVE_BRAKE_GAIN));

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

    public void updateOdometry() {
        pinpoint.update();

        long currentTime = System.currentTimeMillis();
        double currentX = getOdometryX();
        double currentY = getOdometryY();

        if (lastVelocityTime > 0) {
            double deltaTime = (currentTime - lastVelocityTime) / 1000.0;
            if (deltaTime > 0.005) {
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
        return pinpointY;
    }

    public double getOdometryY() {
        double pinpointX = pinpoint.getPosX(DistanceUnit.MM);
        return pinpointX;
    }

    public double getOdometryHeading() {
        double pinpointHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        return pinpointHeading;
    }

    public double getRawIMUHeading() {
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }

    // ========== POSITION PREDICTION ==========

    public double getVelocityMagnitude() {
        return Math.sqrt(velocityX * velocityX + velocityY * velocityY);
    }

    public double getStoppingTime() {
        double speed = getVelocityMagnitude();
        if (speed < 100) return 0;

        double speedRatio = Math.min(1.0, speed / FULL_SPEED_VELOCITY);
        return STOPPING_TIME_HALF_SPEED + speedRatio * (STOPPING_TIME_FULL_SPEED - STOPPING_TIME_HALF_SPEED);
    }

    public double getPredictedX() {
        if (positionOverrideActive) {
            return overrideX;
        }
        double stoppingTime = getStoppingTime();
        double predictedDistance = velocityX * stoppingTime / 2.0;
        return getOdometryX() + predictedDistance;
    }

    public double getPredictedY() {
        if (positionOverrideActive) {
            return overrideY;
        }
        double stoppingTime = getStoppingTime();
        double predictedDistance = velocityY * stoppingTime / 2.0;
        return getOdometryY() + predictedDistance;
    }

    // ========== POSITION OVERRIDE ==========

    public void setPositionOverride(double xInches, double yInches) {
        positionOverrideActive = true;
        overrideX = xInches * INCHES_TO_MM;
        overrideY = yInches * INCHES_TO_MM;
    }

    public void clearPositionOverride() {
        positionOverrideActive = false;
    }

    public boolean isPositionOverrideActive() {
        return positionOverrideActive;
    }

    public double getVelocityX() {
        return velocityX;
    }

    public double getVelocityY() {
        return velocityY;
    }

    public void resetOdometry() {
        pinpoint.resetPosAndIMU();
    }

    public void recalibrateIMU() {
        pinpoint.recalibrateIMU();
    }

    public void setOdometryPosition(double fieldX, double fieldY, double fieldHeading) {

        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, fieldX, fieldY, AngleUnit.DEGREES, fieldHeading));
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
