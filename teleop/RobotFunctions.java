package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;

/**
 * Aggregator class for all robot mechanism functions.
 * Each mechanism initializes its own hardware.
 */
public class RobotFunctions {
    
    // Shared Limelight (used by multiple systems)
    private Limelight3A limelight;
    
    // Sub-function classes
    public DrivetrainFunctions drivetrain;
    public ShooterFunctions shooter;
    public TurretFunctions turret;
    public IntakeFunctions intake;
    public IndexerFunctions indexer;
    public ClimberFunctions climber;
    public LEDFunctions led;
    
    // Alliance tracking
    private boolean isRedAlliance = true;
    
    public RobotFunctions(HardwareMap hardwareMap) {
        // Initialize Limelight (shared between shooter and turret)
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        
        // Initialize all sub-function classes
        drivetrain = new DrivetrainFunctions(hardwareMap);
        shooter = new ShooterFunctions(hardwareMap, limelight);
        turret = new TurretFunctions(hardwareMap, limelight, drivetrain);
        intake = new IntakeFunctions(hardwareMap);
        indexer = new IndexerFunctions(hardwareMap);
        climber = new ClimberFunctions(hardwareMap);
        led = new LEDFunctions(hardwareMap);
    }
    
    // ========== ALLIANCE ==========
    
    public void setAlliance(boolean isRed) {
        this.isRedAlliance = isRed;
        drivetrain.setAlliance(isRed);
    }
    
    public boolean isRedAlliance() {
        return isRedAlliance;
    }
    
    // ========== LIMELIGHT ==========
    
    public void setLimelightPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }
    
    public Limelight3A getLimelight() {
        return limelight;
    }
    
    // ========== UTILITY FUNCTIONS ==========
    
    public boolean eitherGamepad(Gamepad gamepad1, Gamepad gamepad2, String buttonName) {
        boolean pad1 = false;
        boolean pad2 = false;
        
        switch (buttonName.toLowerCase()) {
            case "a":
                pad1 = gamepad1.a;
                pad2 = gamepad2.a;
                break;
            case "b":
                pad1 = gamepad1.b;
                pad2 = gamepad2.b;
                break;
            case "x":
                pad1 = gamepad1.x;
                pad2 = gamepad2.x;
                break;
            case "y":
                pad1 = gamepad1.y;
                pad2 = gamepad2.y;
                break;
            case "left_bumper":
                pad1 = gamepad1.left_bumper;
                pad2 = gamepad2.left_bumper;
                break;
            case "right_bumper":
                pad1 = gamepad1.right_bumper;
                pad2 = gamepad2.right_bumper;
                break;
        }
        
        return pad1 || pad2;
    }
    
    // ========== SERVO INITIALIZATION ==========
    
    /**
     * Initialize all servos to their default positions.
     * Call this AFTER pinpoint has reset to avoid motion during IMU calibration.
     */
    public void initAllServos() {
        turret.initServos();
        intake.initServos();
        indexer.initServos();
        climber.initServos();
    }
    
    // ========== STOP ALL ==========
    
    public void stopAll() {
        drivetrain.stopMotors();
        shooter.stopShooter();
        intake.stopIntakeSystem();
    }
}
