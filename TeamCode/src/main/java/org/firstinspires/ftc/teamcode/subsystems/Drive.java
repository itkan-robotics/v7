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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.teleop.RobotConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import java.util.List;

/**
 * Drive subsystem - combines Pedro Pathing for autonomous and mecanum drive for teleop
 */
public class Drive {

    // ========== PEDRO PATHING (AUTONOMOUS) ==========
    private Follower follower;
    private Pose2D startingPose = new Pose2D(DistanceUnit.INCH, 95, 70, AngleUnit.DEGREES, 45);
    private List<LynxModule> allHubs;
    private ElapsedTime loopTimer = new ElapsedTime();
    private double lastLoopTimeMs = 0;

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

        // Initialize Pedro Pathing Follower using Constants builder
        follower = Constants.createFollower(hardwareMap);
        // Starting pose will be set by the OpMode based on alliance selection
        
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
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(RobotConstants.getPinpointOffsetX(),
                RobotConstants.getPinpointOffsetY(), DistanceUnit.MM);
        pinpoint.setEncoderResolution(34.31, DistanceUnit.MM);
        pinpoint.setEncoderDirections(RobotConstants.getEncoderDirectionX(),
                RobotConstants.getEncoderDirectionY());
        pinpoint.setYawScalar(RobotConstants.getYawScalar());
    }

    /**
     * Re-apply pinpoint settings based on current RobotConstants.
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

//        return isRedAlliance ? ROBOT_START_X_RED : ROBOT_START_X_BLUE;'
        return PoseStorage.x;
    }

    public double getRobotStartY() {

//        return isRedAlliance ? ROBOT_START_Y_RED : ROBOT_START_Y_BLUE;
        return PoseStorage.y;
    }


    public double getRobotStartHeading() {

//        return isRedAlliance ? ROBOT_START_HEADING_RED : ROBOT_START_HEADING_BLUE;
        return PoseStorage.heading;
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
        double pinpointX = getRobotStartY() - fieldY;
        double pinpointY = fieldX - getRobotStartX();
        double pinpointHeading = fieldHeading - getRobotStartHeading();

        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, pinpointX, pinpointY, AngleUnit.DEGREES, pinpointHeading));
    }

    public String getPinpointStatus() {
        return pinpoint.getDeviceStatus().toString();
    }

    public double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    // ========== PEDRO PATHING (AUTONOMOUS) ==========

    public void setStartingPose(Pose2D pose) {
        this.startingPose = pose;
        follower.setStartingPose(new Pose(
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.RADIANS)
        ));
    }

    public Follower getFollower() {
        return follower;
    }

    public void update() {
        follower.update();
    }

    public void setMaxPower(double power) {
        follower.setMaxPower(power);
    }

    public void followPathChain(PathChain pathChain, boolean holdEnd) {
        follower.followPath(pathChain, holdEnd);
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    public Pose2D getCurrentPose() {
        Pose pose = follower.getPose();
        return new Pose2D(
                DistanceUnit.INCH, pose.getX(), pose.getY(),
                AngleUnit.RADIANS, pose.getHeading()
        );
    }

    public void startLoopTimer() {
        loopTimer.reset();
    }

    public double endLoopTimer() {
        lastLoopTimeMs = loopTimer.milliseconds();
        return lastLoopTimeMs;
    }

    public double getLoopTimeMs() {
        return lastLoopTimeMs;
    }

    public void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}
