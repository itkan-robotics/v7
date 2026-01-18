package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Turret Servo Testing Program
 * 
 * Controls:
 * A - Go to 0 degrees (center)
 * B - Go to 90 degrees (right)
 * X - Go to -90 degrees (left)
 * Y - Go to current custom angle
 * 
 * D-Pad Up/Down - Adjust center point offset (fine tune)
 * D-Pad Left/Right - Adjust custom target angle
 * 
 * Left Bumper - Decrease servo range (degrees per full servo travel)
 * Right Bumper - Increase servo range (degrees per full servo travel)
 * 
 * Left Trigger - Fine adjust center point down (hold for continuous)
 * Right Trigger - Fine adjust center point up (hold for continuous)
 * 
 * Left Stick Y - Manual servo position control (direct)
 * Right Stick Y - Manual angle adjustment (continuous)
 */
@TeleOp(name="Turret Test", group="Testing")
public class TurretTest extends OpMode {
    
    private RobotFunctions robot;
    
    // Tuning parameters (start with calibrated values)
    private double centerPosition = TurretFunctions.TURRET_CENTER_POSITION;
    private double degreesPerServoUnit = TurretFunctions.TURRET_DEGREES_PER_SERVO_UNIT;
    
    // Current target angle
    private double customAngle = 45.0;
    private double currentTargetAngle = 0.0;
    
    // Adjustment increments
    private static final double CENTER_COARSE_INCREMENT = 0.01;
    private static final double RANGE_INCREMENT = 5.0;
    private static final double ANGLE_INCREMENT = 5.0;
    private static final double TRIGGER_SPEED = 0.001;
    private static final double STICK_SPEED = 0.5;
    
    // Button debounce
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    
    // Mode tracking
    private enum TurretMode {
        PRESET,
        MANUAL_POS,
        MANUAL_ANGLE
    }
    private TurretMode currentMode = TurretMode.PRESET;
    
    @Override
    public void init() {
        // Initialize robot functions (initializes all hardware)
        robot = new RobotFunctions(hardwareMap);
        
        // Start at center position
        robot.turret.setTurretHome();
        currentTargetAngle = 0.0;
        
        telemetry.addData("Status", "Turret Test Initialized");
        telemetry.addData("", "A=0°  B=90°  X=-90°  Y=Custom");
        telemetry.addData("", "D-Pad: Adjust center/angle");
        telemetry.addData("", "Bumpers: Adjust range");
        telemetry.update();
    }
    
    @Override
    public void loop() {
        // ========== BUTTON PRESSES ==========
        boolean aPressed = gamepad1.a && !lastA;
        boolean bPressed = gamepad1.b && !lastB;
        boolean xPressed = gamepad1.x && !lastX;
        boolean yPressed = gamepad1.y && !lastY;
        boolean dpadUpPressed = gamepad1.dpad_up && !lastDpadUp;
        boolean dpadDownPressed = gamepad1.dpad_down && !lastDpadDown;
        boolean dpadLeftPressed = gamepad1.dpad_left && !lastDpadLeft;
        boolean dpadRightPressed = gamepad1.dpad_right && !lastDpadRight;
        boolean leftBumperPressed = gamepad1.left_bumper && !lastLeftBumper;
        boolean rightBumperPressed = gamepad1.right_bumper && !lastRightBumper;
        
        // Update last button states
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        lastLeftBumper = gamepad1.left_bumper;
        lastRightBumper = gamepad1.right_bumper;
        
        // ========== PRESET ANGLES (A/B/X/Y) ==========
        if (aPressed) {
            currentTargetAngle = 0.0;
            setTurretAngle(currentTargetAngle);
            currentMode = TurretMode.PRESET;
        }
        
        if (bPressed) {
            currentTargetAngle = 90.0;
            setTurretAngle(currentTargetAngle);
            currentMode = TurretMode.PRESET;
        }
        
        if (xPressed) {
            currentTargetAngle = -90.0;
            setTurretAngle(currentTargetAngle);
            currentMode = TurretMode.PRESET;
        }
        
        if (yPressed) {
            currentTargetAngle = customAngle;
            setTurretAngle(currentTargetAngle);
            currentMode = TurretMode.PRESET;
        }
        
        // ========== CENTER POINT ADJUSTMENT (D-Pad Up/Down) ==========
        if (dpadUpPressed) {
            centerPosition += CENTER_COARSE_INCREMENT;
            if (centerPosition > 1.0) centerPosition = 1.0;
            setTurretAngle(currentTargetAngle);
        }
        
        if (dpadDownPressed) {
            centerPosition -= CENTER_COARSE_INCREMENT;
            if (centerPosition < 0.0) centerPosition = 0.0;
            setTurretAngle(currentTargetAngle);
        }
        
        // ========== CUSTOM ANGLE ADJUSTMENT (D-Pad Left/Right) ==========
        if (dpadLeftPressed) {
            customAngle -= ANGLE_INCREMENT;
        }
        
        if (dpadRightPressed) {
            customAngle += ANGLE_INCREMENT;
        }
        
        // ========== SERVO RANGE ADJUSTMENT (Bumpers) ==========
        if (leftBumperPressed) {
            degreesPerServoUnit -= RANGE_INCREMENT;
            if (degreesPerServoUnit < 180.0) degreesPerServoUnit = 180.0;
            setTurretAngle(currentTargetAngle);
        }
        
        if (rightBumperPressed) {
            degreesPerServoUnit += RANGE_INCREMENT;
            if (degreesPerServoUnit > 500.0) degreesPerServoUnit = 500.0;
            setTurretAngle(currentTargetAngle);
        }
        
        // ========== FINE CENTER ADJUSTMENT (Triggers) ==========
        if (gamepad1.left_trigger > 0.1) {
            centerPosition -= TRIGGER_SPEED * gamepad1.left_trigger;
            if (centerPosition < 0.0) centerPosition = 0.0;
            setTurretAngle(currentTargetAngle);
        }
        
        if (gamepad1.right_trigger > 0.1) {
            centerPosition += TRIGGER_SPEED * gamepad1.right_trigger;
            if (centerPosition > 1.0) centerPosition = 1.0;
            setTurretAngle(currentTargetAngle);
        }
        
        // ========== MANUAL SERVO POSITION (Left Stick Y) ==========
        double leftStickY = -gamepad1.left_stick_y;
        if (Math.abs(leftStickY) > 0.1) {
            double newPosition = centerPosition + (leftStickY * 0.5);
            robot.turret.setTurretPosition(newPosition);
            currentMode = TurretMode.MANUAL_POS;
        }
        
        // ========== MANUAL ANGLE ADJUSTMENT (Right Stick Y) ==========
        double rightStickY = -gamepad1.right_stick_y;
        if (Math.abs(rightStickY) > 0.1) {
            currentTargetAngle += rightStickY * STICK_SPEED;
            double servoPos = centerPosition + (currentTargetAngle / degreesPerServoUnit);
            if (servoPos > TurretFunctions.TURRET_MAX_SERVO) {
                servoPos = TurretFunctions.TURRET_MAX_SERVO;
                currentTargetAngle = (servoPos - centerPosition) * degreesPerServoUnit;
            }
            if (servoPos < TurretFunctions.TURRET_MIN_SERVO) {
                servoPos = TurretFunctions.TURRET_MIN_SERVO;
                currentTargetAngle = (servoPos - centerPosition) * degreesPerServoUnit;
            }
            setTurretAngle(currentTargetAngle);
            currentMode = TurretMode.MANUAL_ANGLE;
        }
        
        // ========== TELEMETRY ==========
        telemetry.addData("=== TURRET TEST ===", "");
        telemetry.addData("Mode", currentMode.toString());
        telemetry.addData("", "");
        
        telemetry.addData("=== CURRENT STATE ===", "");
        telemetry.addData("Target Angle", "%.1f°", currentTargetAngle);
        telemetry.addData("Servo Cmd/Est", "%.4f / %.4f", 
            robot.turret.getTurretServoPosition(), 
            robot.turret.getEstimatedServoPosition());
        telemetry.addData("Calculated Angle", "%.1f°", 
            calculateAngleFromPosition(robot.turret.getTurretServoPosition()));
        telemetry.addData("Servo Limits", "%.3f to %.3f", 
            TurretFunctions.TURRET_MIN_SERVO, TurretFunctions.TURRET_MAX_SERVO);
        telemetry.addData("Servo Speed", "%.3f units/sec", TurretFunctions.TURRET_SERVO_SPEED);
        telemetry.addData("", "");
        
        telemetry.addData("=== TUNING PARAMETERS ===", "");
        telemetry.addData("Center Position", "%.4f (D-Pad ↑↓ or Triggers)", centerPosition);
        telemetry.addData("Deg/Servo Unit", "%.1f° (Bumpers L/R)", degreesPerServoUnit);
        telemetry.addData("Custom Angle", "%.0f° (D-Pad ←→)", customAngle);
        telemetry.addData("", "");
        
        telemetry.addData("=== CALIBRATED POSITIONS ===", "");
        telemetry.addData("0° (center)", "%.4f", TurretFunctions.TURRET_CENTER_POSITION);
        telemetry.addData("+90° (field 180°)", "%.4f", TurretFunctions.TURRET_90_POSITION);
        telemetry.addData("-90° (field 0°)", "%.4f", TurretFunctions.TURRET_NEG90_POSITION);
        telemetry.addData("", "");
        
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("Presets", "A=0°  B=90°  X=-90°  Y=%.0f°", customAngle);
        telemetry.addData("Sticks", "L=Direct Pos  R=Angle Adjust");
        telemetry.addData("", "");
        
        telemetry.addData("=== CONFIG VALUES ===", "");
        telemetry.addData("Copy to TurretFunctions:", "");
        telemetry.addData("  TURRET_CENTER_POSITION", "%.4f", centerPosition);
        telemetry.addData("  TURRET_DEGREES_PER_SERVO_UNIT", "%.1f", degreesPerServoUnit);
        
        telemetry.update();
    }
    
    private double angleToServoPosition(double turretAngle) {
        double servoPosition = centerPosition + (turretAngle / degreesPerServoUnit);
        if (servoPosition < 0.0) servoPosition = 0.0;
        if (servoPosition > 1.0) servoPosition = 1.0;
        return servoPosition;
    }
    
    private double calculateAngleFromPosition(double servoPosition) {
        return (servoPosition - centerPosition) * degreesPerServoUnit;
    }
    
    private void setTurretAngle(double angle) {
        double servoPosition = angleToServoPosition(angle);
        robot.turret.setTurretPosition(servoPosition);
    }
    
    @Override
    public void stop() {
        robot.turret.setTurretHome();
    }
}
