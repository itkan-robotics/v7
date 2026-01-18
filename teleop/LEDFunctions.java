package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * LED/Light servo functions
 * Self-contained with its own hardware initialization
 */
public class LEDFunctions {
    
    // ========== HARDWARE ==========
    private ServoImplEx lightServo;
    
    // ========== CONSTANTS ==========
    public static final double LIGHT_OFF = 0.0;
    public static final double LIGHT_RED = 0.305;
    public static final double LIGHT_ORANGE = 0.333;
    public static final double LIGHT_YELLOW = 0.388;
    public static final double LIGHT_GREEN = 0.5;
    public static final double LIGHT_BLUE = 0.666;
    public static final double LIGHT_PURPLE = 0.722;
    public static final double LIGHT_WHITE = 1.0;
    
    public LEDFunctions(HardwareMap hardwareMap) {
        // Initialize RGB light servo
        lightServo = hardwareMap.get(ServoImplEx.class, "light_servo");
        lightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }
    
    // ========== LED CONTROL ==========
    
    public void updateLightServo(boolean shooterRunning, boolean shooterReady, boolean intakeRunning, 
                                  boolean isShooting, boolean aprilTagVisible, boolean turretOnTarget, 
                                  boolean turretUsingVisualTracking, boolean hasThreeBalls) {
        double targetColor;
        
        if (isShooting && shooterReady && turretOnTarget) {
            // Ready to fire AND actively shooting (highest priority)
            targetColor = LIGHT_WHITE;
        }
        else if (turretOnTarget && shooterReady) {
            // Ready to fire
            targetColor = LIGHT_BLUE;
        }
        else if (hasThreeBalls) {
            // Robot has 3 balls loaded (current * voltage > 62 after startup)
            targetColor = LIGHT_PURPLE;
        }
        else if (turretOnTarget && shooterRunning) {
            targetColor = LIGHT_GREEN;
        }
        else if (aprilTagVisible && turretUsingVisualTracking) {
            targetColor = LIGHT_YELLOW;
        }
        else {
            targetColor = LIGHT_RED;
        }
        
        lightServo.setPosition(targetColor);
    }
    
    public void setLightPosition(double position) {
        lightServo.setPosition(position);
    }
    
    public void setLightOff() {
        lightServo.setPosition(LIGHT_OFF);
    }
    
    public void setLightRed() {
        lightServo.setPosition(LIGHT_RED);
    }
    
    public void setLightOrange() {
        lightServo.setPosition(LIGHT_ORANGE);
    }
    
    public void setLightYellow() {
        lightServo.setPosition(LIGHT_YELLOW);
    }
    
    public void setLightGreen() {
        lightServo.setPosition(LIGHT_GREEN);
    }
    
    public void setLightBlue() {
        lightServo.setPosition(LIGHT_BLUE);
    }
    
    public void setLightPurple() {
        lightServo.setPosition(LIGHT_PURPLE);
    }
    
    public void setLightWhite() {
        lightServo.setPosition(LIGHT_WHITE);
    }
    
    public double getLightPosition() {
        return lightServo.getPosition();
    }
}
