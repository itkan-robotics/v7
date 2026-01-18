package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Climber servo functions
 * Self-contained with its own hardware initialization
 */
public class ClimberFunctions {
    
    // ========== HARDWARE ==========
    private Servo climberServo;
    
    // ========== CONSTANTS ==========
    // Climber positions come from RobotConstants
    
    public ClimberFunctions(HardwareMap hardwareMap) {
        // Initialize climber servo (don't set position until initServos() is called)
        climberServo = hardwareMap.get(Servo.class, "climber_servo");
    }
    
    /**
     * Initialize servo to default position. Call after pinpoint has reset.
     */
    public void initServos() {
        climberServo.setPosition(RobotConstants.getClimberDown());
    }
    
    // ========== CLIMBER CONTROL ==========
    
    public void setClimberDown() {
        climberServo.setPosition(RobotConstants.getClimberDown());
    }
    
    public void setClimberUp() {
        climberServo.setPosition(RobotConstants.getClimberUp());
    }
    
    public void setClimberPosition(double position) {
        climberServo.setPosition(position);
    }
    
    public double getClimberPosition() {
        return climberServo.getPosition();
    }
    
    public boolean isClimberDown() {
        double pos = climberServo.getPosition();
        return Math.abs(pos - RobotConstants.getClimberDown()) < 0.05;
    }
    
    public boolean isClimberUp() {
        double pos = climberServo.getPosition();
        return Math.abs(pos - RobotConstants.getClimberUp()) < 0.05;
    }
}
