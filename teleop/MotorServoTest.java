package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Motor and Servo Testing Program
 * Updated with turret servo, odometry, and sensor testing
 * 
 * Controls:
 * D-Pad Up/Down - Select motor/servo
 * A - Test selected device forward/increase position
 * B - Test selected device reverse/decrease position
 * X - Stop all motors
 * Y - Reset to default positions
 * Left Bumper - Previous device
 * Right Bumper - Next device
 * Left Stick Y - Variable speed control for selected motor
 * Right Stick Y - Fine servo adjustment (when servo selected)
 * 
 * Left Stick Button - Reset odometry position to field center
 * Right Stick Button - Reset IMU heading
 */
@TeleOp(name="Motor Servo Test", group="Testing")
public class MotorServoTest extends OpMode {
    
    private RobotFunctions robot;
    
    // Test device selection
    private int selectedDevice = 0;
    private final String[] deviceNames = {
        "Drivetrain (all)",
        "Shooter Motor",
        "Intake Motor",
        "Transfer Motor",
        "Blocker Servo",
        "Indexing Servo",
        "Climber Servo",
        "Turret Servo",
        "Light Servo (LED)"
    };
    
    // Button debounce
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastLeftStickButton = false;
    private boolean lastRightStickButton = false;
    
    // Servo positions (initial values will be set in init() after RobotConstants is configured)
    private double blockerPosition = 0.5;
    private double indexingPosition = 0.5;
    private double climberPosition = 0.5;
    private double turretPosition = TurretFunctions.TURRET_CENTER_POSITION;
    private double lightPosition = 0.5;
    
    private final double SERVO_INCREMENT = 0.05;
    private final double SERVO_FINE_INCREMENT = 0.01;
    private final double TEST_POWER = 0.3;
    
    @Override
    public void init() {
        // Default to robot 21171 for testing (can be changed via RobotConstants if needed)
        RobotConstants.setRobot(RobotConstants.ROBOT_21171);
        
        // Initialize robot functions (initializes all hardware, servos not positioned yet)
        robot = new RobotFunctions(hardwareMap);
        
        // Reset pinpoint first
        robot.drivetrain.resetOdometry();
        
        // Initialize all servos to default positions
        robot.initAllServos();
        
        // Track servo positions
        blockerPosition = RobotConstants.getBlockerBlocked();
        indexingPosition = RobotConstants.getIndexerMiddle();
        climberPosition = RobotConstants.getClimberDown();
        turretPosition = TurretFunctions.TURRET_CENTER_POSITION;
        
        robot.led.setLightGreen();
        lightPosition = LEDFunctions.LIGHT_GREEN;
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("", "Use D-Pad or Bumpers to select device");
        telemetry.addData("", "A=Forward/Inc, B=Reverse/Dec, X=Stop All, Y=Reset");
        telemetry.update();
    }
    
    @Override
    public void loop() {
        // Update odometry
        robot.drivetrain.updateOdometry();
        
        // ========== DEVICE SELECTION ==========
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        boolean leftBumper = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;
        
        if ((dpadUp && !lastDpadUp) || (leftBumper && !lastLeftBumper)) {
            selectedDevice--;
            if (selectedDevice < 0) selectedDevice = deviceNames.length - 1;
        }
        if ((dpadDown && !lastDpadDown) || (rightBumper && !lastRightBumper)) {
            selectedDevice++;
            if (selectedDevice >= deviceNames.length) selectedDevice = 0;
        }
        
        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;
        lastLeftBumper = leftBumper;
        lastRightBumper = rightBumper;
        
        // ========== SPECIAL CONTROLS ==========
        if (gamepad1.left_stick_button && !lastLeftStickButton) {
            robot.drivetrain.resetPositionToFieldCenter();
            telemetry.addData("Action", "Position reset to field center!");
        }
        lastLeftStickButton = gamepad1.left_stick_button;
        
        if (gamepad1.right_stick_button && !lastRightStickButton) {
            robot.drivetrain.recalibrateIMU();
            telemetry.addData("Action", "IMU heading reset!");
        }
        lastRightStickButton = gamepad1.right_stick_button;
        
        // ========== DEVICE CONTROL ==========
        boolean aPressed = gamepad1.a && !lastA;
        boolean bPressed = gamepad1.b && !lastB;
        boolean xPressed = gamepad1.x && !lastX;
        boolean yPressed = gamepad1.y && !lastY;
        
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        
        // Stop all motors
        if (xPressed) {
            robot.stopAll();
            telemetry.addData("Action", "All motors stopped");
        }
        
        // Reset to defaults
        if (yPressed) {
            robot.stopAll();
            robot.intake.setBlocker(true);
            blockerPosition = RobotConstants.getBlockerBlocked();
            robot.indexer.setIndexerMiddle();
            indexingPosition = RobotConstants.getIndexerMiddle();
            robot.climber.setClimberPosition(0.5);
            climberPosition = 0.5;
            robot.turret.setTurretHome();
            turretPosition = TurretFunctions.TURRET_CENTER_POSITION;
            robot.led.setLightGreen();
            lightPosition = LEDFunctions.LIGHT_GREEN;
            telemetry.addData("Action", "Reset to defaults");
        }
        
        double leftStickY = -gamepad1.left_stick_y;
        double rightStickY = -gamepad1.right_stick_y;
        
        // Control selected device
        switch (selectedDevice) {
            case 0: // Drivetrain
                if (Math.abs(leftStickY) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1) {
                    robot.drivetrain.mecanumDrive(leftStickY, gamepad1.left_stick_x, 
                        -gamepad1.right_stick_x, 0.5);
                } else {
                    robot.drivetrain.stopMotors();
                }
                break;
                
            case 1: // Shooter
                if (gamepad1.a) {
                    robot.shooter.controlShooter(true);
                } else {
                    robot.shooter.stopShooter();
                }
                break;
                
            case 2: // Intake
                if (aPressed || Math.abs(leftStickY) > 0.1) {
                    robot.intake.setIntakePower(Math.abs(leftStickY) > 0.1 ? leftStickY : TEST_POWER);
                } else if (bPressed) {
                    robot.intake.setIntakePower(-TEST_POWER);
                }
                break;
                
            case 3: // Transfer
                if (aPressed || Math.abs(leftStickY) > 0.1) {
                    robot.intake.setTransferPower(Math.abs(leftStickY) > 0.1 ? leftStickY : TEST_POWER);
                } else if (bPressed) {
                    robot.intake.setTransferPower(-TEST_POWER);
                }
                break;
                
            case 4: // Blocker Servo
                if (aPressed) blockerPosition += SERVO_INCREMENT;
                if (bPressed) blockerPosition -= SERVO_INCREMENT;
                if (Math.abs(rightStickY) > 0.1) blockerPosition += rightStickY * SERVO_FINE_INCREMENT;
                blockerPosition = Math.max(0.0, Math.min(1.0, blockerPosition));
                robot.intake.setBlockerPosition(blockerPosition);  // Use actual position, not boolean
                break;
                
            case 5: // Indexing Servo
                if (aPressed) indexingPosition += SERVO_INCREMENT;
                if (bPressed) indexingPosition -= SERVO_INCREMENT;
                if (Math.abs(rightStickY) > 0.1) indexingPosition += rightStickY * SERVO_FINE_INCREMENT;
                indexingPosition = Math.max(0.0, Math.min(1.0, indexingPosition));
                robot.indexer.setIndexerPosition(indexingPosition);
                break;
                
            case 6: // Climber Servo
                if (aPressed) climberPosition += SERVO_INCREMENT;
                if (bPressed) climberPosition -= SERVO_INCREMENT;
                if (Math.abs(rightStickY) > 0.1) climberPosition += rightStickY * SERVO_FINE_INCREMENT;
                climberPosition = Math.max(0.0, Math.min(1.0, climberPosition));
                robot.climber.setClimberPosition(climberPosition);
                break;
                
            case 7: // Turret Servo
                if (aPressed) turretPosition += SERVO_INCREMENT;
                if (bPressed) turretPosition -= SERVO_INCREMENT;
                if (Math.abs(rightStickY) > 0.1) turretPosition += rightStickY * SERVO_FINE_INCREMENT;
                turretPosition = Math.max(TurretFunctions.TURRET_MIN_SERVO, 
                    Math.min(TurretFunctions.TURRET_MAX_SERVO, turretPosition));
                robot.turret.setTurretPosition(turretPosition);
                break;
                
            case 8: // Light Servo (LED)
                if (aPressed) lightPosition += SERVO_INCREMENT;
                if (bPressed) lightPosition -= SERVO_INCREMENT;
                if (Math.abs(rightStickY) > 0.1) lightPosition += rightStickY * SERVO_FINE_INCREMENT;
                lightPosition = Math.max(0.0, Math.min(1.0, lightPosition));
                robot.led.setLightPosition(lightPosition);
                break;
        }
        
        // ========== TELEMETRY ==========
        telemetry.addData("=== DEVICE SELECTION ===", "");
        telemetry.addData("Selected", "[%d] %s", selectedDevice, deviceNames[selectedDevice]);
        telemetry.addData("Controls", "A=Inc B=Dec X=Stop Y=Reset");
        
        telemetry.addData("=== SHOOTER ===", "");
        telemetry.addData("TPS", "%.0f / %.0f", 
            robot.shooter.getShooterTPS(), robot.shooter.getTargetShooterTPS());
        
        telemetry.addData("=== SERVOS ===", "");
        telemetry.addData("Blocker", "%.3f", blockerPosition);
        telemetry.addData("Indexing", "%.3f", indexingPosition);
        telemetry.addData("Climber", "%.3f", climberPosition);
        
        double turretAngle = robot.turret.getTurretAngle();
        telemetry.addData("Turret", "%.3f (%.1f°) est:%.3f", turretPosition, turretAngle, 
            robot.turret.getEstimatedServoPosition());
        telemetry.addData("Light (LED)", "%.3f", lightPosition);
        
        telemetry.addData("=== ODOMETRY ===", "");
        telemetry.addData("Field Position", "X:%.1f Y:%.1f", 
            robot.drivetrain.getOdometryX() / 25.4, 
            robot.drivetrain.getOdometryY() / 25.4);
        telemetry.addData("Heading", "%.1f° (IMU: %.1f°)", 
            robot.drivetrain.getOdometryHeading(),
            robot.drivetrain.getRawIMUHeading());
        telemetry.addData("Pinpoint Status", robot.drivetrain.getPinpointStatus());
        
        telemetry.addData("=== LIMELIGHT ===", "");
        boolean isRed = robot.isRedAlliance();
        int tagId = robot.turret.getDetectedAprilTagId(isRed);
        telemetry.addData("AprilTag", tagId > 0 ? "ID " + tagId : "None");
        if (tagId > 0) {
            telemetry.addData("Tag tx/ty", "%.1f° / %.1f°", 
                robot.turret.getLimelightTx(isRed), robot.turret.getLimelightTy());
            telemetry.addData("Tag Distance", "%.1f in", 
                robot.turret.getAprilTagDistance(isRed) / 25.4);
        }
        
        telemetry.addData("=== TRANSFER / 3-BALL ===", "");
        telemetry.addData("Transfer Current", "%.2f A", robot.intake.getTransferCurrent());
        telemetry.addData("Battery Voltage", "%.2f V", robot.intake.getBatteryVoltage());
        telemetry.addData("Power (I×V)", "%.1f (threshold: 62)", robot.intake.getTransferPowerValue());
        telemetry.addData("3 Balls Detected", robot.intake.hasThreeBalls() ? "YES" : "NO");
        
        telemetry.addData("=== ENCODERS ===", "");
        telemetry.addData("FL/FR", "%d / %d", 
            robot.drivetrain.getFrontLeftPosition(), robot.drivetrain.getFrontRightPosition());
        telemetry.addData("BL/BR", "%d / %d", 
            robot.drivetrain.getBackLeftPosition(), robot.drivetrain.getBackRightPosition());
        
        telemetry.addData("", "");
        telemetry.addData("LeftStickBtn", "Reset to field center");
        telemetry.addData("RightStickBtn", "Reset IMU heading");
        
        telemetry.update();
    }
    
    @Override
    public void stop() {
        robot.stopAll();
    }
}
