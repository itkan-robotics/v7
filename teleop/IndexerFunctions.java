package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Indexer servo functions
 * Self-contained with its own hardware initialization
 */
public class IndexerFunctions {
    
    // ========== HARDWARE ==========
    private Servo indexingServo;
    
    // ========== CONSTANTS ==========
    // Indexer positions come from RobotConstants
    
    public IndexerFunctions(HardwareMap hardwareMap) {
        // Initialize indexing servo (don't set position until initServos() is called)
        indexingServo = hardwareMap.get(Servo.class, "indexing_servo");
    }
    
    /**
     * Initialize servo to default position. Call after pinpoint has reset.
     */
    public void initServos() {
        indexingServo.setPosition(RobotConstants.getIndexerMiddle());
    }
    
    // ========== INDEXER CONTROL ==========
    
    public void setIndexerIndexed() {
        indexingServo.setPosition(RobotConstants.getIndexerIndexed());
    }
    
    public void setIndexerMiddle() {
        indexingServo.setPosition(RobotConstants.getIndexerMiddle());
    }
    
    public void setIndexerPosition(double position) {
        indexingServo.setPosition(position);
    }
    
    public double getIndexerPosition() {
        return indexingServo.getPosition();
    }
    
    public boolean isIndexerAtMiddle() {
        double currentPos = indexingServo.getPosition();
        return Math.abs(currentPos - RobotConstants.getIndexerMiddle()) < 0.05;
    }
    
    public boolean isIndexerAtIndexed() {
        double currentPos = indexingServo.getPosition();
        return Math.abs(currentPos - RobotConstants.getIndexerIndexed()) < 0.05;
    }
}
