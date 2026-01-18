package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Intake, transfer, and blocker functions
 * Self-contained with its own hardware initialization
 */
public class IntakeFunctions {
    
    // ========== HARDWARE ==========
    private DcMotor intakeMotor;
    private DcMotorEx transferMotor;  // DcMotorEx for current monitoring
    private Servo blockerServo;
    private VoltageSensor voltageSensor;
    
    // ========== CONSTANTS ==========
    // Blocker positions come from RobotConstants
    
    // Three ball detection constants
    // Formula: transferCurrent * batteryVoltage > 62
    public static final double THREE_BALL_POWER_THRESHOLD = 62.0;  // Current * Voltage threshold
    public static final double TRANSFER_STARTUP_IGNORE_TIME = 500;  // Milliseconds
    
    // ========== STATE ==========
    private boolean lastBlockerState = true;  // Track last state to avoid redundant commands
    // Negative power = toward shooter (motors are set to REVERSE direction)
    public static final double INTAKE_POWER = -1.0;
    public static final double TRANSFER_POWER = -1.0;
    public static final double TRANSFER_POWER_FAR = -0.65;
    public static final double APRILTAG_AREA_CLOSE_THRESHOLD = 0.5;
    
    // Transfer motor timing for 3-ball detection
    private long transferStartTime = 0;
    private boolean transferWasRunning = false;
    
    public IntakeFunctions(HardwareMap hardwareMap) {
        // Initialize intake motor
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize transfer motor (DcMotorEx for current monitoring)
        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer_motor");
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize blocker servo (don't set position until initServos() is called)
        blockerServo = hardwareMap.get(Servo.class, "blocker_servo");
        
        // Initialize voltage sensor for 3-ball detection
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }
    
    /**
     * Initialize servo to default position. Call after pinpoint has reset.
     */
    public void initServos() {
        blockerServo.setPosition(RobotConstants.getBlockerBlocked());
        lastBlockerState = true;
    }
    
    // ========== INTAKE/TRANSFER CONTROL ==========
    
    public void runIntakeSystem(double power) {
        intakeMotor.setPower(power);
        transferMotor.setPower(power);
    }
    
    public void runIntakeSystemShooting(double area) {
        intakeMotor.setPower(INTAKE_POWER);
        transferMotor.setPower(getTransferSpeed(area));
    }
    
    public void stopIntakeSystem() {
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
    }
    
    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }
    
    public void setTransferPower(double power) {
        // Track when transfer motor starts running for 3-ball detection
        boolean isRunning = Math.abs(power) > 0.05;
        if (isRunning && !transferWasRunning) {
            // Motor just started - record time
            transferStartTime = System.currentTimeMillis();
        }
        transferWasRunning = isRunning;
        
        transferMotor.setPower(power);
    }
    
    // ========== THREE BALL DETECTION ==========
    
    /**
     * Check if the robot likely has 3 balls loaded.
     * Returns true if (transferCurrent * batteryVoltage) > 62 AND motor has been running > 0.5 sec
     * (ignores startup current spike)
     */
    public boolean hasThreeBalls() {
        // Must be running
        if (!transferWasRunning) {
            return false;
        }
        
        // Check if enough time has passed since motor started (ignore startup spike)
        long timeSinceStart = System.currentTimeMillis() - transferStartTime;
        if (timeSinceStart < TRANSFER_STARTUP_IGNORE_TIME) {
            return false;
        }
        
        // Check current * voltage > threshold
        double currentAmps = transferMotor.getCurrent(CurrentUnit.AMPS);
        double voltage = voltageSensor.getVoltage();
        double power = currentAmps * voltage;
        return power > THREE_BALL_POWER_THRESHOLD;
    }
    
    /**
     * Get the current draw of the transfer motor in amps
     */
    public double getTransferCurrent() {
        return transferMotor.getCurrent(CurrentUnit.AMPS);
    }
    
    /**
     * Get the battery voltage
     */
    public double getBatteryVoltage() {
        return voltageSensor.getVoltage();
    }
    
    /**
     * Get the power value used for 3-ball detection (current * voltage)
     */
    public double getTransferPowerValue() {
        return transferMotor.getCurrent(CurrentUnit.AMPS) * voltageSensor.getVoltage();
    }
    
    public double getTransferSpeed(double area) {
        if (area >= APRILTAG_AREA_CLOSE_THRESHOLD) {
            return TRANSFER_POWER;
        } else {
            return TRANSFER_POWER_FAR;
        }
    }
    
    // ========== BLOCKER CONTROL ==========
    
    /**
     * Set blocker position - only sends command if state changed
     * This prevents servo jitter from constant re-commanding
     */
    public void setBlocker(boolean blocked) {
        // Only update if state changed
        if (blocked != lastBlockerState) {
            if (blocked) {
                blockerServo.setPosition(RobotConstants.getBlockerBlocked());
            } else {
                blockerServo.setPosition(RobotConstants.getBlockerUnblocked());
            }
            lastBlockerState = blocked;
        }
    }
    
    /**
     * Force set blocker position regardless of state
     * Use this for initialization or when you need to ensure position
     */
    public void forceSetBlocker(boolean blocked) {
        if (blocked) {
            blockerServo.setPosition(RobotConstants.getBlockerBlocked());
        } else {
            blockerServo.setPosition(RobotConstants.getBlockerUnblocked());
        }
        lastBlockerState = blocked;
    }
    
    /**
     * Set blocker to a specific position value
     */
    public void setBlockerPosition(double position) {
        blockerServo.setPosition(position);
        lastBlockerState = (position < 0.5);  // Estimate state based on position
    }
    
    public double getBlockerPosition() {
        return blockerServo.getPosition();
    }
    
    public boolean isBlocked() {
        return lastBlockerState;
    }
}
