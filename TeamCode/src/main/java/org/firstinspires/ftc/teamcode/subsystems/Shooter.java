package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants.RobotConstants;

/**
 * Shooter subsystem - combines shooter motor, turret, intake/transfer, and LED control
 */
public class Shooter {

    // ========== HARDWARE ==========
    private DcMotorEx shooterMotor;
    private DcMotorEx intakeTransferMotor;  // Single motor for intake and transfer
    private Servo blockerServo;
    // TURRET SERVO REMOVED - Now using motor instead
    private Servo climberServo;
    private ServoImplEx lightServo;
    private VoltageSensor voltageSensor;
    private Limelight3A limelight;

    // Reference to drive for odometry-based turret pointing
    private Drive drive;

    // ========== SHOOTER CONSTANTS ==========
    public static final double SHOOTER_MAX_POWER = 1.0;
    public static final double SHOOTER_DEFAULT_TPS = 1750.0;
    public static final double SHOOTER_MIN_TPS = 1250.0;
    public static final double SHOOTER_MAX_TPS = 2000.0;
    public static final double SHOOTER_READY_THRESHOLD = 50.0;
    
    // Backward compatibility constants for autonomous
    public static final double DEFAULT_TARGET_SHOOTER_VELOCITY = 1350.0;
    public static final double VELOCITY_TOLERANCE = 50.0;
    public static final double INTAKE_POWER = 1.0;
    public static final double LIMELIGHT_TOLERANCE = 1.5;
    public static final double LIMELIGHT_KP = 0.035;

    // ========== TURRET CONSTANTS ==========
    // TURRET SERVO CONSTANTS REMOVED - Now using motor instead

    // Goal positions (mm)
    public static final double INCHES_TO_MM = 25.4;
    public static final double GOAL_RED_X = 138.0 * INCHES_TO_MM;
    public static final double GOAL_RED_Y = 143.5 * INCHES_TO_MM;
    public static final double GOAL_BLUE_X = 6.0 * INCHES_TO_MM;
    public static final double GOAL_BLUE_Y = 143.5 * INCHES_TO_MM;

    // ========== INTAKE CONSTANTS ==========
    public static final double THREE_BALL_POWER_THRESHOLD = 62.0;
    public static final double TRANSFER_STARTUP_IGNORE_TIME = 500;
    
    // Intake full detection thresholds
    public static final double INTAKE_CURRENT_THRESHOLD = 2.0;
    public static final double INTAKE_VELOCITY_THRESHOLD = 50.0;
    public static final double INTAKE_STALL_VELOCITY = 10.0;
    public static final double INTAKE_POWER_THRESHOLD = 0.8;
    
    // Intake states
    public enum IntakeState {
        IDLE,
        HOLD,
        INTAKING,
        OUTTAKING
    }
    private IntakeState currentIntakeState = IntakeState.IDLE;

    // ========== LED CONSTANTS ==========
    public static final double LIGHT_OFF = 0.0;
    public static final double LIGHT_RED = 0.305;
    public static final double LIGHT_ORANGE = 0.333;
    public static final double LIGHT_YELLOW = 0.388;
    public static final double LIGHT_GREEN = 0.5;
    public static final double LIGHT_BLUE = 0.666;
    public static final double LIGHT_PURPLE = 0.722;
    public static final double LIGHT_WHITE = 1.0;

    // ========== STATE ==========
    private double lastShooterPower = 0.0;
    private double overrideDefaultTPS = 0.0;
    private boolean lastBlockerState = true;

    // Turret state
    // TURRET SERVO STATE REMOVED - Now using motor instead

    // Cached limelight data
    private LLResult cachedLimelightResult = null;
    private double cachedTx = 0.0;
    private double cachedTy = 0.0;
    private double cachedTa = 0.0;
    private int cachedTagId = -1;
    private boolean cachedHasTarget = false;
    private double cachedTagDistance = 0.0;

    // Transfer motor timing for 3-ball detection
    private long transferStartTime = 0;
    private boolean transferWasRunning = false;
    
    // Indexer servo (for autonomous compatibility)
    private Servo indexingServo;
    
    // Indexer constants
    public static final double INDEXER_INDEXED = 0.05;
    public static final double INDEXER_MIDDLE = 0.55;

    /**
     * Constructor for autonomous (without Drive reference - turret uses manual angles only)
     */
    public Shooter(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    /**
     * Constructor for teleop (with Drive reference for odometry-based turret tracking)
     */
    public Shooter(HardwareMap hardwareMap, Drive drive) {
        this.drive = drive;

        // Initialize voltage sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Initialize shooter motor
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooterMotor.setDirection(RobotConstants.getShooterDirection());
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize intake/transfer motor (single motor)
        intakeTransferMotor = hardwareMap.get(DcMotorEx.class, "transfer_motor");
        intakeTransferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeTransferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize blocker servo
        blockerServo = hardwareMap.get(Servo.class, "blocker_servo");

        // TURRET SERVO INITIALIZATION REMOVED - Now using motor instead

        // Initialize climber servo
        climberServo = hardwareMap.get(Servo.class, "climber_servo");

        // Initialize LED servo
        lightServo = hardwareMap.get(ServoImplEx.class, "light_servo");
        lightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        
        // Initialize indexer servo
        indexingServo = hardwareMap.get(Servo.class, "indexing_servo");
    }

    /**
     * Re-apply motor settings based on current RobotConstants.
     */
    public void applyMotorSettings() {
        shooterMotor.setDirection(RobotConstants.getShooterDirection());
    }

    /**
     * Initialize all servos to default positions. Call after pinpoint has reset.
     */
    public void initServos() {
        // TURRET SERVO INITIALIZATION REMOVED - Now using motor instead

        blockerServo.setPosition(RobotConstants.getBlockerBlocked());
        lastBlockerState = true;

        climberServo.setPosition(RobotConstants.getClimberDown());
        
        indexingServo.setPosition(RobotConstants.getIndexerMiddle());
    }

    // ========== SHOOTER MOTOR CONTROL ==========

    public double getShooterTPS() {
        return Math.abs(shooterMotor.getVelocity());
    }

    public double getTargetShooterTPS() {
        if (!cachedHasTarget) {
            return (overrideDefaultTPS > 0) ? overrideDefaultTPS : SHOOTER_DEFAULT_TPS;
        }
        return getTargetShooterTPS(cachedTy, true);
    }

    public double getTargetShooterTPS(double ty, boolean hasTarget) {
        if (!hasTarget) {
            return (overrideDefaultTPS > 0) ? overrideDefaultTPS : SHOOTER_DEFAULT_TPS;
        }

        double targetTPS;

        double TY_1 = RobotConstants.getLimelightTy1();
        double TY_2 = RobotConstants.getLimelightTy2();
        double TY_3 = RobotConstants.getLimelightTy3();
        double TY_4 = RobotConstants.getLimelightTy4();
        double TY_5 = RobotConstants.getLimelightTy5();
        double TY_6 = RobotConstants.getLimelightTy6();
        double TPS_1 = RobotConstants.getShooterTps1();
        double TPS_2 = RobotConstants.getShooterTps2();
        double TPS_3 = RobotConstants.getShooterTps3();
        double TPS_4 = RobotConstants.getShooterTps4();
        double TPS_5 = RobotConstants.getShooterTps5();
        double TPS_6 = RobotConstants.getShooterTps6();

        if (ty >= TY_1) {
            targetTPS = TPS_1;
        } else if (ty >= TY_2) {
            double tyRange = TY_1 - TY_2;
            double tpsRange = TPS_2 - TPS_1;
            double normalized = (TY_1 - ty) / tyRange;
            targetTPS = TPS_1 + (normalized * tpsRange);
        } else if (ty >= TY_3) {
            double tyRange = TY_2 - TY_3;
            double tpsRange = TPS_3 - TPS_2;
            double normalized = (TY_2 - ty) / tyRange;
            targetTPS = TPS_2 + (normalized * tpsRange);
        } else if (ty >= TY_4) {
            double tyRange = TY_3 - TY_4;
            double tpsRange = TPS_4 - TPS_3;
            double normalized = (TY_3 - ty) / tyRange;
            targetTPS = TPS_3 + (normalized * tpsRange);
        } else if (ty >= TY_5) {
            double tyRange = TY_4 - TY_5;
            double tpsRange = TPS_5 - TPS_4;
            double normalized = (TY_4 - ty) / tyRange;
            targetTPS = TPS_4 + (normalized * tpsRange);
        } else if (ty >= TY_6) {
            double tyRange = TY_5 - TY_6;
            double tpsRange = TPS_6 - TPS_5;
            double normalized = (TY_5 - ty) / tyRange;
            targetTPS = TPS_5 + (normalized * tpsRange);
        } else {
            targetTPS = TPS_6;
        }

        if (targetTPS < SHOOTER_MIN_TPS) targetTPS = SHOOTER_MIN_TPS;
        if (targetTPS > SHOOTER_MAX_TPS) targetTPS = SHOOTER_MAX_TPS;

        return targetTPS;
    }

    public void controlShooter(boolean running) {
        controlShooter(running, false);
    }

    public void controlShooter(boolean running, boolean isActivelyFeeding) {
        if (!running) {
            shooterMotor.setPower(0);
            lastShooterPower = 0.0;
            return;
        }

        double targetTPS = getTargetShooterTPS();
        double currentTPS = getShooterTPS();

        double power = (currentTPS < targetTPS) ? SHOOTER_MAX_POWER : 0.0;

        shooterMotor.setPower(power);
        lastShooterPower = power;
    }

    public void controlShooterManual(double targetTPS, boolean running) {
        controlShooterManual(targetTPS, running, false);
    }

    public void controlShooterManual(double targetTPS, boolean running, boolean isActivelyFeeding) {
        if (!running) {
            shooterMotor.setPower(0);
            lastShooterPower = 0.0;
            return;
        }

        double currentTPS = getShooterTPS();
        double effectiveTarget = targetTPS;
        if (isActivelyFeeding && targetTPS > 1700) {
            effectiveTarget += 100;
        }

        double power = (currentTPS < effectiveTarget) ? SHOOTER_MAX_POWER : 0.0;

        shooterMotor.setPower(power);
        lastShooterPower = power;
    }

    public void stopShooter() {
        shooterMotor.setPower(0);
        lastShooterPower = 0.0;
    }

    public double getShooterPower() {
        return lastShooterPower;
    }

    public void setDefaultTPSOverride(double tps) {
        overrideDefaultTPS = tps;
    }

    public void clearDefaultTPSOverride() {
        overrideDefaultTPS = 0.0;
    }

    public boolean isShooterSpeedReady(double targetTPS) {
        double currentTPS = getShooterTPS();
        return Math.abs(currentTPS - targetTPS) <= SHOOTER_READY_THRESHOLD;
    }
    
    // Backward compatibility methods for autonomous
    public double getShooterVelocity() {
        return getShooterTPS();
    }
    
    /**
     * Bang-bang velocity control for autonomous
     */
    public void updateShooter(boolean enable, double targetVelocity) {
        if (!enable) {
            shooterMotor.setPower(0);
            lastShooterPower = 0.0;
            return;
        }
        
        double currentVelocity = getShooterTPS();
        double power = (currentVelocity < targetVelocity) ? SHOOTER_MAX_POWER : 0.0;
        
        shooterMotor.setPower(power);
        lastShooterPower = power;
    }
    
    /**
     * Check if intake is full (backward compatibility)
     */
    public boolean issintakeFull() {
        return getPowerConsumption() > 75;
    }
    
    /**
     * Alias for issintakeFull for backward compatibility
     */
    public boolean intakeFull() {
        return hasThreeBalls();
    }
    
    /**
     * Check if shooter is ready (backward compatibility for autonomous)
     */
    public boolean isShooterReady(double targetVelocity, boolean isAligned) {
        double currentVelocity = getShooterTPS();
        boolean speedReady = Math.abs(currentVelocity - targetVelocity) <= VELOCITY_TOLERANCE;
        return speedReady && isAligned;
    }
    
    /**
     * Get power consumption for debugging (backward compatibility)
     */
    public double getPowerConsumption() {
        double currentAmps = intakeTransferMotor.getCurrent(CurrentUnit.AMPS);
        double voltage = voltageSensor.getVoltage();
        return currentAmps * voltage;
    }

    // ========== TURRET CONTROL ==========
    // TURRET SERVO CODE REMOVED - Now using motor instead
    // All turret servo methods have been removed and replaced with placeholder comments

    // ========== LIMELIGHT FUNCTIONS ==========

    public void updateLimelightData(boolean isRedAlliance) {
        cachedLimelightResult = limelight.getLatestResult();

        cachedTx = 0.0;
        cachedTy = 0.0;
        cachedTa = 0.0;
        cachedTagId = -1;
        cachedHasTarget = false;
        cachedTagDistance = 0.0;

        if (cachedLimelightResult != null && cachedLimelightResult.isValid()) {
            cachedHasTarget = true;
            cachedTx = cachedLimelightResult.getTx();
            cachedTy = cachedLimelightResult.getTy();
            cachedTa = cachedLimelightResult.getTa();

            int targetTagId = isRedAlliance ? 24 : 20;
            LLResultTypes.FiducialResult goalTag = findTargetTag(targetTagId);

            if (goalTag != null) {
                cachedTagId = (int) goalTag.getFiducialId();

                org.firstinspires.ftc.robotcore.external.navigation.Pose3D cameraPose = goalTag.getTargetPoseCameraSpace();
                if (cameraPose != null) {
                    double x = cameraPose.getPosition().x;
                    double y = cameraPose.getPosition().y;
                    double z = cameraPose.getPosition().z;
                    cachedTagDistance = Math.sqrt(x * x + y * y + z * z);
                }
            }
        }
    }

    private LLResultTypes.FiducialResult findTargetTag(int targetTagId) {
        if (cachedLimelightResult != null && cachedLimelightResult.isValid()) {
            if (cachedLimelightResult.getFiducialResults() != null && !cachedLimelightResult.getFiducialResults().isEmpty()) {
                for (LLResultTypes.FiducialResult fiducial : cachedLimelightResult.getFiducialResults()) {
                    if ((int) fiducial.getFiducialId() == targetTagId) {
                        return fiducial;
                    }
                }
            }
        }
        return null;
    }

    public int getDetectedAprilTagId(boolean isRedAlliance) {
        return cachedTagId;
    }

    public double getAprilTagArea() {
        return cachedTa;
    }

    public double getAprilTagDistance(boolean isRedAlliance) {
        return cachedTagDistance;
    }

    public double getLimelightTx(boolean isRedAlliance) {
        if (cachedTagId > 0) {
            return cachedTx;
        }
        return 0.0;
    }

    public double getLimelightTy() {
        return cachedTy;
    }

    public boolean hasLimelightTarget() {
        return cachedHasTarget;
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    // ========== INTAKE/TRANSFER CONTROL ==========

    public void runIntakeSystem(double power) {
        intakeTransferMotor.setPower(power);
    }

    public void stopIntakeSystem() {
        intakeTransferMotor.setPower(0);
    }

    public void setIntakePower(double power) {
        boolean isRunning = Math.abs(power) > 0.05;
        if (isRunning && !transferWasRunning) {
            transferStartTime = System.currentTimeMillis();
        }
        transferWasRunning = isRunning;

        intakeTransferMotor.setPower(power);
    }

    public void setTransferPower(double power) {
        setIntakePower(power);  // Same motor now
    }

    public boolean hasThreeBalls() {
        if (!transferWasRunning) {
            return false;
        }

        long timeSinceStart = System.currentTimeMillis() - transferStartTime;
        if (timeSinceStart < TRANSFER_STARTUP_IGNORE_TIME) {
            return false;
        }

        double currentAmps = intakeTransferMotor.getCurrent(CurrentUnit.AMPS);
        double voltage = voltageSensor.getVoltage();
        double power = currentAmps * voltage;
        return power > THREE_BALL_POWER_THRESHOLD;
    }

    public double getTransferCurrent() {
        return intakeTransferMotor.getCurrent(CurrentUnit.AMPS);
    }

    public double getBatteryVoltage() {
        return voltageSensor.getVoltage();
    }

    // ========== BLOCKER CONTROL ==========

    public void setBlocker(boolean blocked) {
        if (blocked != lastBlockerState) {
            if (blocked) {
                blockerServo.setPosition(RobotConstants.getBlockerBlocked());
            } else {
                blockerServo.setPosition(RobotConstants.getBlockerUnblocked());
            }
            lastBlockerState = blocked;
        }
    }

    public boolean isBlocked() {
        return lastBlockerState;
    }
    
    // Backward compatibility methods for autonomous
    public void blockShooter() {
        setBlocker(true);
    }
    
    public void unblockShooter() {
        setBlocker(false);
    }

    // ========== INDEXER CONTROL ==========
    
    public void setIndexerIndexed() {
        indexingServo.setPosition(RobotConstants.getIndexerIndexed());
    }
    
    public void setIndexerMiddle() {
        indexingServo.setPosition(RobotConstants.getIndexerMiddle());
    }
    
    public double getIndexerPosition() {
        return indexingServo.getPosition();
    }

    // ========== CLIMBER CONTROL ==========

    public void setClimberDown() {
        climberServo.setPosition(RobotConstants.getClimberDown());
    }

    public void setClimberUp() {
        climberServo.setPosition(RobotConstants.getClimberUp());
    }

    // ========== LED CONTROL ==========

    public void updateLightServo(boolean shooterRunning, boolean shooterReady, boolean intakeRunning,
                                  boolean isShooting, boolean aprilTagVisible, boolean turretOnTarget,
                                  boolean turretUsingVisualTracking, boolean hasThreeBalls) {
        double targetColor;

        if (isShooting && shooterReady && turretOnTarget) {
            targetColor = LIGHT_WHITE;
        } else if (turretOnTarget && shooterReady) {
            targetColor = LIGHT_BLUE;
        } else if (hasThreeBalls) {
            targetColor = LIGHT_PURPLE;
        } else if (turretOnTarget && shooterRunning) {
            targetColor = LIGHT_GREEN;
        } else if (aprilTagVisible && turretUsingVisualTracking) {
            targetColor = LIGHT_YELLOW;
        } else {
            targetColor = LIGHT_RED;
        }

        lightServo.setPosition(targetColor);
    }

    public void setLightOff() {
        lightServo.setPosition(LIGHT_OFF);
    }

    // ========== STOP ALL ==========

    public void stopAll() {
        stopShooter();
        stopIntakeSystem();
    }
}
