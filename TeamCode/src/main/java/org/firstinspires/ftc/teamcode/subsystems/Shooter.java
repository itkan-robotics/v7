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
import com.qualcomm.robotcore.robot.Robot;

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
    private DcMotorEx turretMotor;

    private Servo blockerServo;
    private Servo climberServo;
    private ServoImplEx lightServo;
    private VoltageSensor voltageSensor;
    private Limelight3A limelight;

    // Reference to drive for odometry-based turret pointing

    // Intake states
    public enum IntakeState {
        IDLE,
        HOLD,
        INTAKING,
        OUTTAKING
    }
    private IntakeState currentIntakeState = IntakeState.IDLE;

    // ========== STATE ==========
    private double lastShooterPower = 0.0;
    private double overrideDefaultTPS = 0.0;
    private boolean lastBlockerState = true;

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
    
    // Turret state
    private double targetTxOffset = 1.0;
    private double lastVisualError = 0.0;
    
    // Turret PID override values (for runtime tuning via Panels)
    private Double turretKpOverride = null;
    private Double turretVisualKpOverride = null;
    private Double turretVisualKdOverride = null;
    private Double turretVisualKfOverride = null;
    private Double turretTurnFfOverride = null;
    
    // Turret zeroing state machine
    private enum TurretZeroState {
        IDLE,
        DRIVING_TO_HARDSTOP,
        COMPLETE
    }
    private TurretZeroState turretZeroState = TurretZeroState.IDLE;
    private long turretZeroStartTime = 0;
    private static final double TURRET_ZERO_POWER = -0.5;
    private static final long TURRET_ZERO_MIN_TIME_MS = 200;
    private static final double TURRET_ZERO_VELOCITY_THRESHOLD = 5.0;



    /**
     * Constructor for autonomous (without Drive reference - turret uses manual angles only)
     */


    /**
     * Constructor for teleop (with Drive reference for odometry-based turret tracking)
     */
    public Shooter(HardwareMap hardwareMap) {

        // Initialize voltage sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        
        // Default to tracking no specific tags until alliance is set
        limelight.updatePythonInputs(new double[]{0, 0, 0, 0, 0, 0, 0, 0});

        // Initialize shooter motor
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooterMotor.setDirection(RobotConstants.getShooterDirection());
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TURRET MOTOR COMMENTED OUT - Controlled directly in TurretMotorTuning to avoid double hardware init
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
            return (overrideDefaultTPS > 0) ? overrideDefaultTPS : RobotConstants.SHOOTER_DEFAULT_TPS;
        }
        return getTargetShooterTPS(cachedTy, true);
    }

    public double getTargetShooterTPS(double ty, boolean hasTarget) {
        if (!hasTarget) {
            return (overrideDefaultTPS > 0) ? overrideDefaultTPS : RobotConstants.SHOOTER_DEFAULT_TPS;
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

        if (targetTPS < RobotConstants.SHOOTER_MIN_TPS) targetTPS = RobotConstants.SHOOTER_MIN_TPS;
        if (targetTPS > RobotConstants.SHOOTER_MAX_TPS) targetTPS = RobotConstants.SHOOTER_MAX_TPS;

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

        double power = (currentTPS < targetTPS) ? RobotConstants.SHOOTER_MAX_POWER : 0.0;

        shooterMotor.setPower(power);
        lastShooterPower = power;
    }
    
    // ========== TURRET MOTOR CONTROL ==========
    
    /**
     * Convert a turret angle (in degrees) to motor encoder ticks.
     * 0 degrees = turret aligned with intake direction
     * Positive angle = counter-clockwise rotation
     * 
     * @param angleDegrees The target angle in degrees
     * @return The motor position in encoder ticks
     */
    /**
     * Convert turret angle (degrees) to motor ticks.
     * Accounts for hardstop deadzone: 0 ticks = 5°, max ticks = 355°.
     * 
     * @param angleDegrees Turret angle in degrees (-180 to 180 from calculateTurretAngleToGoal)
     * @return Motor ticks clamped to valid range
     */
    public double angleToTurretTicks(double angleDegrees) {
        // Normalize angle to 0-360 range
        double normalizedAngle = angleDegrees;
        while (normalizedAngle < 0) normalizedAngle += 360;
        while (normalizedAngle >= 360) normalizedAngle -= 360;
        
        // Clamp to valid turret range (5° to 355°)
        if (normalizedAngle < RobotConstants.TURRET_MIN_ANGLE) {
            normalizedAngle = RobotConstants.TURRET_MIN_ANGLE;
        } else if (normalizedAngle > RobotConstants.TURRET_MAX_ANGLE) {
            normalizedAngle = RobotConstants.TURRET_MAX_ANGLE;
        }
        
        // Convert to ticks: 0 ticks = 5°, so subtract the min angle offset
        double ticks = (normalizedAngle - RobotConstants.TURRET_MIN_ANGLE) * RobotConstants.TURRET_TICKS_PER_DEGREE;
        
        return ticks;
    }
    
    /**
     * Get the current turret motor encoder position.
     * COMMENTED OUT - Turret motor controlled directly in TurretMotorTuning
     */
    public double getTurretEncoderPos() {
         return turretMotor.getCurrentPosition();
    }
    
    /**
     * Set turret motor power directly.
     * COMMENTED OUT - Turret motor controlled directly in TurretMotorTuning
     */
    public void setTurretPower(double power) {
        // turretMotor.setPower(power);
        // Placeholder - turret controlled in TurretMotorTuning
    }
    
    /**
     * Stop the turret motor.
     */
    public void stopTurret() {
        turretMotor.setPower(0);
    }
    
    /**
     * Start the turret zeroing process. Call this once to begin zeroing.
     */
    public void startTurretZero() {
        turretZeroState = TurretZeroState.DRIVING_TO_HARDSTOP;
        turretZeroStartTime = System.currentTimeMillis();
    }
    
    /**
     * Update the turret zeroing state machine. Call this in a loop during initialization.
     * Drives turret at low negative power until it hits hardstop (velocity = 0),
     * then resets encoder.
     * 
     * @return true if zeroing is complete, false if still in progress
     */
    public boolean updateTurretZero() {
        switch (turretZeroState) {
            case IDLE:
                return true;
                
            case DRIVING_TO_HARDSTOP:
                turretMotor.setPower(TURRET_ZERO_POWER);
                
                long elapsed = System.currentTimeMillis() - turretZeroStartTime;
                double velocity = Math.abs(turretMotor.getVelocity());
                
                if (elapsed > TURRET_ZERO_MIN_TIME_MS && velocity < TURRET_ZERO_VELOCITY_THRESHOLD) {
                    turretMotor.setPower(0);
                    turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    turretZeroState = TurretZeroState.COMPLETE;
                }
                return false;
                
            case COMPLETE:
                return true;
                
            default:
                return true;
        }
    }
    
    /**
     * Check if turret zeroing is complete.
     */
    public boolean isTurretZeroComplete() {
        return turretZeroState == TurretZeroState.COMPLETE || turretZeroState == TurretZeroState.IDLE;
    }
    
    /**
     * Reset turret zero state to allow re-zeroing.
     */
    public void resetTurretZeroState() {
        turretZeroState = TurretZeroState.IDLE;
    }


    public void pointTurretAtGoal(boolean isRedAlliance, boolean allowVisualTracking, double targetAngle) {
        pointTurretAtGoal(isRedAlliance, allowVisualTracking, targetAngle, 0.0);
    }
    
    public void pointTurretAtGoal(boolean isRedAlliance, boolean allowVisualTracking, double targetAngle, double turnInput) {
        int expectedTagId = isRedAlliance ? 24 : 20;
        int detectedTagId = getDetectedAprilTagId(isRedAlliance);
        if (allowVisualTracking && detectedTagId == expectedTagId) {
            pointTurretVisual(isRedAlliance, turnInput);
        } else {
            pointTurretByPosition(isRedAlliance, targetAngle, turnInput);
        }
    }

    /**
     * Set the target TX offset for visual tracking.
     * @param offset The desired TX offset in degrees
     */
    public void setTargetTxOffset(double offset) {
        targetTxOffset = offset;
    }
    
    /**
     * Get the current target TX offset.
     */
    public double getTargetTxOffset() {
        return targetTxOffset;
    }
    
    // ========== TURRET PID OVERRIDE SETTERS (for Panels tuning) ==========
    
    /**
     * Set override for turret position Kp (null to use RobotConstants default)
     */
    public void setTurretKpOverride(Double kp) {
        turretKpOverride = kp;
    }
    
    /**
     * Set override for turret visual Kp (null to use RobotConstants default)
     */
    public void setTurretVisualKpOverride(Double kp) {
        turretVisualKpOverride = kp;
    }
    
    /**
     * Set override for turret visual Kd (null to use RobotConstants default)
     */
    public void setTurretVisualKdOverride(Double kd) {
        turretVisualKdOverride = kd;
    }
    
    /**
     * Set override for turret visual Kf (null to use RobotConstants default)
     */
    public void setTurretVisualKfOverride(Double kf) {
        turretVisualKfOverride = kf;
    }
    
    /**
     * Set override for turret turn feedforward (null to use RobotConstants default)
     */
    public void setTurretTurnFfOverride(Double ff) {
        turretTurnFfOverride = ff;
    }
    
    /**
     * Set all turret PID overrides at once (pass null for any value to use default)
     */
    public void setTurretPidOverrides(Double kp, Double visualKp, Double visualKd, Double visualKf, Double turnFf) {
        turretKpOverride = kp;
        turretVisualKpOverride = visualKp;
        turretVisualKdOverride = visualKd;
        turretVisualKfOverride = visualKf;
        turretTurnFfOverride = turnFf;
    }
    
    // Helper methods to get effective PID values (override or default)
    private double getEffectiveTurretKp() {
        return turretKpOverride != null ? turretKpOverride : RobotConstants.TURRET_KP;
    }
    
    private double getEffectiveVisualKp() {
        return turretVisualKpOverride != null ? turretVisualKpOverride : RobotConstants.TURRET_VISUAL_KP;
    }
    
    private double getEffectiveVisualKd() {
        return turretVisualKdOverride != null ? turretVisualKdOverride : RobotConstants.TURRET_VISUAL_KD;
    }
    
    private double getEffectiveVisualKf() {
        return turretVisualKfOverride != null ? turretVisualKfOverride : RobotConstants.TURRET_VISUAL_KF;
    }
    
    private double getEffectiveTurnFf() {
        return turretTurnFfOverride != null ? turretTurnFfOverride : RobotConstants.TURRET_TURN_FF;
    }

    public void pointTurretVisual(boolean isRedAlliance, double turnInput) {
         double tx = getLimelightTx(isRedAlliance);
         double error = targetTxOffset - tx;
         double derivative = error - lastVisualError;
         lastVisualError = error;
         if(Math.abs(error) < 0.25) {
             turretMotor.setPower(0.0);
             return;
         }
         
         double pTerm = error * getEffectiveVisualKp();
         double dTerm = derivative * getEffectiveVisualKd();
         double ffTerm = turnInput * getEffectiveTurnFf();
         double kfTerm = Math.signum(error) * getEffectiveVisualKf();
         double power = Math.max(-0.5, Math.min(0.5, pTerm + dTerm + ffTerm + kfTerm));
         turretMotor.setPower(power);
    }
    public void pointTurretVisual(boolean isRedAlliance) {
        double tx = getLimelightTx(isRedAlliance);
        double error = targetTxOffset - tx;
        double derivative = error - lastVisualError;
        lastVisualError = error;
        if(Math.abs(error) < 0.25) {
            turretMotor.setPower(0.0);
            return;
        }

        double pTerm = error * getEffectiveVisualKp();
        double dTerm = derivative * getEffectiveVisualKd();
        double kfTerm = Math.signum(error) * getEffectiveVisualKf();
        double power = Math.max(-0.5, Math.min(0.5, pTerm + dTerm + kfTerm));
        turretMotor.setPower(power);
    }

    public void pointTurretByPosition(boolean isRedAlliance, double targetAngleDegrees, double turnInput) {
         double targetTicks = angleToTurretTicks(targetAngleDegrees);
         double currentTicks = turretMotor.getCurrentPosition();
         double errorTicks = targetTicks - currentTicks;

         double pTerm = errorTicks * getEffectiveTurretKp();
         double ffTerm = turnInput * getEffectiveTurnFf();
         double power = Math.max(-1.0, Math.min(1.0, pTerm + ffTerm));
         turretMotor.setPower(power);
    }
    public void pointTurretByPosition(double targetAngleDegrees) {
        double targetTicks = angleToTurretTicks(targetAngleDegrees);
        double currentTicks = turretMotor.getCurrentPosition();
        double errorTicks = targetTicks - currentTicks;

        double pTerm = errorTicks * getEffectiveTurretKp();
        double power = Math.max(-1.0, Math.min(1.0, pTerm));
        turretMotor.setPower(power);
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

        double power = (currentTPS < effectiveTarget) ? RobotConstants.SHOOTER_MAX_POWER : 0.0;

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
        return Math.abs(currentTPS - targetTPS) <= RobotConstants.SHOOTER_READY_THRESHOLD;
    }
    
    // Backward compatibility methods for autonomous
    public double getShooterVelocity() {
        return getShooterTPS();
    }
    public double getTurretPower() { return turretMotor.getPower(); }
    
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
        double power = (currentVelocity < targetVelocity) ? RobotConstants.SHOOTER_MAX_POWER : 0.0;
        
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
        boolean speedReady = Math.abs(currentVelocity - targetVelocity) <= RobotConstants.VELOCITY_TOLERANCE;
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
            
            // Find the specific goal tag for our alliance
            int targetTagId = isRedAlliance ? 24 : 20;
            LLResultTypes.FiducialResult goalTag = findTargetTag(targetTagId);

            if (goalTag != null) {
                // Use tx/ty from the SPECIFIC goal tag, not the primary result
                // This ensures we ignore incorrect tags when multiple are visible
                cachedTagId = (int) goalTag.getFiducialId();
                cachedTx = goalTag.getTargetXDegrees();
                cachedTy = goalTag.getTargetYDegrees();
                cachedTa = goalTag.getTargetArea();

                org.firstinspires.ftc.robotcore.external.navigation.Pose3D cameraPose = goalTag.getTargetPoseCameraSpace();
                if (cameraPose != null) {
                    double x = cameraPose.getPosition().x;
                    double y = cameraPose.getPosition().y;
                    double z = cameraPose.getPosition().z;
                    cachedTagDistance = Math.sqrt(x * x + y * y + z * z);
                }
            } else {
                // No correct tag found - use primary result for ty (shooter speed) only
                // but leave cachedTagId as -1 so visual tracking won't be used
                cachedTy = cachedLimelightResult.getTy();
                cachedTa = cachedLimelightResult.getTa();
            }
        }
    }

    /**
     * Get a comma-separated string of all detected tag IDs for debugging.
     */
    public String getAllDetectedTagIds() {
        if (cachedLimelightResult == null || !cachedLimelightResult.isValid()) {
            return "none";
        }
        java.util.List<LLResultTypes.FiducialResult> fiducials = cachedLimelightResult.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return "none";
        }
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < fiducials.size(); i++) {
            if (i > 0) sb.append(",");
            sb.append((int) fiducials.get(i).getFiducialId());
        }
        return sb.toString();
    }
    
    /**
     * Get the primary (first) tag's tx for debugging comparison.
     */
    public double getPrimaryTx() {
        if (cachedLimelightResult != null && cachedLimelightResult.isValid()) {
            return cachedLimelightResult.getTx();
        }
        return 0.0;
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
    
    /**
     * Invalidate cached limelight data. Call when not actively updating limelight
     * to prevent stale data from being used for visual tracking.
     */
    public void invalidateLimelightCache() {
        cachedHasTarget = false;
        cachedTagId = -1;
        cachedTx = 0.0;
        cachedTy = 0.0;
        cachedTa = 0.0;
        cachedTagDistance = 0.0;
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
        if (timeSinceStart < RobotConstants.TRANSFER_STARTUP_IGNORE_TIME) {
            return false;
        }

        double currentAmps = intakeTransferMotor.getCurrent(CurrentUnit.AMPS);
        double voltage = voltageSensor.getVoltage();
        double power = currentAmps * voltage;
        return power > RobotConstants.THREE_BALL_POWER_THRESHOLD;
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
            targetColor = RobotConstants.LIGHT_WHITE;
        } else if (turretOnTarget && shooterReady) {
            targetColor = RobotConstants.LIGHT_BLUE;
        } else if (hasThreeBalls) {
            targetColor = RobotConstants.LIGHT_PURPLE;
        } else if (turretOnTarget && shooterRunning) {
            targetColor = RobotConstants.LIGHT_GREEN;
        } else if (aprilTagVisible && turretUsingVisualTracking) {
            targetColor = RobotConstants.LIGHT_YELLOW;
        } else {
            targetColor = RobotConstants.LIGHT_RED;
        }

        lightServo.setPosition(targetColor);
    }

    public void setLightOff() {
        lightServo.setPosition(RobotConstants.LIGHT_OFF);
    }

    // ========== STOP ALL ==========

    public void stopAll() {
        stopShooter();
        stopIntakeSystem();
    }
}
