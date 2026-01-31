package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
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
    private DcMotorEx intakeMotor;
    private DcMotorEx turretMotor;

    // CR servos for intake assist (coaxial with intake motor)
    private CRServo intakeServoL;
    private CRServo intakeServoR;

    private Servo blockerServo;
    private Servo climberServo;
    private ServoImplEx lightServo;
    private VoltageSensor voltageSensor;
    private Limelight3A limelight;

    // ========== STATE ==========
    private double lastShooterPower = 0.0;
    private double overrideDefaultTPS = 0.0;
    private boolean lastBlockerState = true;

    // ========== LIMELIGHT THREADING ==========
    private Thread limelightThread;
    private volatile boolean limelightThreadRunning = false;
    private volatile boolean isRedAllianceForLimelight = true;

    // Cached limelight data (volatile for thread safety)
    private volatile LLResult cachedLimelightResult = null;
    private volatile double cachedTx = 0.0;
    private volatile double cachedTy = 0.0;
    private volatile double cachedTa = 0.0;
    private volatile int cachedTagId = -1;
    private volatile boolean cachedHasTarget = false;
    private volatile double cachedTagDistance = 0.0;
    private volatile long lastLimelightUpdateTime = 0;
    
    // Stale frame detection using limelight's internal timestamp
    private volatile double lastLimelightTimestamp = -1;
    private volatile long lastTimestampChangeTime = 0;

    // Intake motor timing for 3-ball detection
    private long transferStartTime = 0;
    private boolean transferWasRunning = false;
    
    // Throttled 3-ball detection (expensive current + voltage reads)
    private static final long THREE_BALL_CHECK_INTERVAL_MS = 50;
    private long lastThreeBallCheckTime = 0;
    private boolean cachedHasThreeBalls = false;
    
    
    // Turret state
    private double targetTxOffset = 0.0;
    private double lastVisualError = 0.0;
    private double smoothedTurnFF = 0.0;  // For turn FF ramp-down
    private long lastTurretUpdateTime = 0;
    
    // Turret PID override values (for runtime tuning via Panels)
    private Double turretKpOverride = null;
    private Double turretVisualKpOverride = null;
    private Double turretVisualKdOverride = null;
    private Double turretVisualKfOverride = null;
    private Double turretTurnFfOverride = null;
    private Double turretTurnFfDecayOverride = null;
    private Double turretVisualDeadbandOverride = null;
    private Double turretVisualMaxPowerOverride = null;
    private Double turretLimelightThresholdOverride = null;
    
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
    private double turretZeroPowerMultiplier = 1.0;

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

        // Initialize turret motor
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize intake motor
        intakeMotor = hardwareMap.get(DcMotorEx.class, "transfer_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize intake CR servos (coaxial with intake motor)
        intakeServoL = hardwareMap.get(CRServo.class, "intake_servoL");
        intakeServoR = hardwareMap.get(CRServo.class, "intake_servoR");

        // Initialize blocker servo - start in BLOCKED position immediately
        blockerServo = hardwareMap.get(Servo.class, "blocker_servo");
        blockerServo.setPosition(0.35);  // Blocked position (will be updated in initServos for correct robot)
        lastBlockerState = true;

        // Initialize climber servo
        climberServo = hardwareMap.get(Servo.class, "climber_servo");

        // Initialize LED servo
        lightServo = hardwareMap.get(ServoImplEx.class, "light_servo");
        lightServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        
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
        blockerServo.setPosition(RobotConstants.getBlockerBlocked());
        lastBlockerState = true;
        climberServo.setPosition(RobotConstants.getClimberDown());
    }

    // ========== SHOOTER MOTOR CONTROL ==========

    public double getShooterTPS() {
        return Math.abs(shooterMotor.getVelocity());
    }

    /**
     * Get target shooter TPS.
     * - If tag visible: use ty-based calculation for distance accuracy
     * - If no tag: use override if set (bumper presets), otherwise default
     */
    public double getTargetShooterTPS() {
        // If tag visible, always use ty-based calculation for accuracy
        if (cachedHasTarget) {
            return calculateTPSFromTy(cachedTy);
        }
        // No tag - use override if set (bumper presets), otherwise default
        return (overrideDefaultTPS > 0) ? overrideDefaultTPS : RobotConstants.SHOOTER_DEFAULT_TPS;
    }

    /**
     * Calculate TPS from ty value using lookup table interpolation.
     */
    private double calculateTPSFromTy(double ty) {
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
        controlShooter(running, getShooterTPS(), getTargetShooterTPS());
    }

    public void setTurretBrake( ){
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void setTurretFloat( ){
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
    /**
     * Control shooter motor using pre-computed TPS values.
     * Use this overload to avoid redundant TPS calculations.
     */
    public void controlShooter(boolean running, double currentTPS, double targetTPS) {
        if (!running) {
            shooterMotor.setPower(0);
            lastShooterPower = 0.0;
            return;
        }

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
     */
    public double getTurretEncoderPos() {
         return turretMotor.getCurrentPosition();
    }
    
    /**
     * Set turret motor power directly.
     */
    public void setTurretPower(double power) {
        turretMotor.setPower(power);
    }

    public double setTurretPowerSafe(double power) {
        double currentTicks = turretMotor.getCurrentPosition();
        double safePower = power;

        //Check if past soft limits
        boolean atMinLimit = currentTicks <= RobotConstants.TURRET_SOFT_MIN_TICKS;
        boolean atMaxLimit = currentTicks >= RobotConstants.TURRET_SOFT_MAX_TICKS;

        // dont allow power that would drive further into the limit
        if(atMinLimit && power < 0) {
            safePower = 0;
        } else if(atMaxLimit && power > 0) {
            safePower = 0;
        }

        turretMotor.setPower(safePower);
        return safePower;
    }
    
    /**
     * Stop the turret motor.
     */
    public void stopTurret() {
        turretMotor.setPower(0);
    }
    
    // ========== RUN_TO_POSITION TURRET CONTROL ==========
    
    /**
     * Set turret to RUN_TO_POSITION mode for simpler control.
     * Call this once during init if using position mode.
     */
    public void setTurretRunToPositionMode() {
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    /**
     * Set turret back to RUN_WITHOUT_ENCODER mode for custom PID control.
     */
    public void setTurretRunWithoutEncoderMode() {
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    /**
     * Move turret to target position using RUN_TO_POSITION mode.
     * @param targetTicks Target position in encoder ticks
     * @param power Motor power (0.0 to 1.0)
     */
    public void setTurretTargetPosition(int targetTicks, double power) {
        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setPower(power);
    }
    
    /**
     * Check if turret has reached its target position (for RUN_TO_POSITION mode).
     * @param tolerance Tolerance in ticks
     * @return true if within tolerance of target
     */
    public boolean isTurretAtTarget(double tolerance) {
        int current = turretMotor.getCurrentPosition();
        int target = turretMotor.getTargetPosition();
        return Math.abs(current - target) <= tolerance;
    }
    
    /**
     * Adjust turret target position by visual offset (for RUN_TO_POSITION visual tracking).
     * Converts TX degrees to ticks and adjusts the target.
     * @param baseTicks Base target position from odometry
     * @param txDegrees TX error in degrees (positive = target is to the right)
     * @param power Motor power
     */
    public void setTurretTargetWithVisualOffset(int baseTicks, double txDegrees, double power) {
        // Convert TX degrees to ticks offset (negative because positive TX means turn right/negative ticks)
        int txOffsetTicks = (int)(-txDegrees * RobotConstants.TURRET_TICKS_PER_DEGREE);
        int adjustedTarget = baseTicks + txOffsetTicks;
        
        // Clamp to valid range
        adjustedTarget = Math.max(0, Math.min((int)RobotConstants.TURRET_MAX_TICKS, adjustedTarget));
        
        turretMotor.setTargetPosition(adjustedTarget);
        turretMotor.setPower(power);
    }
    
    /**
     * Start the turret zeroing process. Call this once to begin zeroing.
     */
    public void startTurretZero() {
        startTurretZero(1.0);
    }
    
    /**
     * Start the turret zeroing process with a custom power multiplier.
     * @param powerMultiplier multiplier for zero power (e.g., 1.75 for faster zeroing)
     */
    public void startTurretZero(double powerMultiplier) {
        turretZeroPowerMultiplier = powerMultiplier;
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
                turretMotor.setPower(TURRET_ZERO_POWER * turretZeroPowerMultiplier);
                
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
    public boolean isTurretAligned(boolean red){
       double error = targetTxOffset - getLimelightTx(red);
       return Math.abs(error) < 2;

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
    
    private double getEffectiveTurnFfDecay() {
        return turretTurnFfDecayOverride != null ? turretTurnFfDecayOverride : RobotConstants.TURRET_TURN_FF_DECAY;
    }
    
    private double getEffectiveVisualDeadband() {
        return turretVisualDeadbandOverride != null ? turretVisualDeadbandOverride : RobotConstants.TURRET_VISUAL_DEADBAND;
    }
    
    private double getEffectiveVisualMaxPower() {
        return turretVisualMaxPowerOverride != null ? turretVisualMaxPowerOverride : RobotConstants.TURRET_VISUAL_MAX_POWER;
    }
    
    public double getEffectiveLimelightThreshold() {
        return turretLimelightThresholdOverride != null ? turretLimelightThresholdOverride : RobotConstants.TURRET_LIMELIGHT_THRESHOLD;
    }
    
    /**
     * Set all turret tuning overrides at once.
     */
    public void setAllTurretOverrides(Double posKp, Double visKp, Double visKd, Double visKf, 
                                       Double turnFf, Double turnFfDecay, Double visDeadband, 
                                       Double visMaxPower, Double llThreshold) {
        turretKpOverride = posKp;
        turretVisualKpOverride = visKp;
        turretVisualKdOverride = visKd;
        turretVisualKfOverride = visKf;
        turretTurnFfOverride = turnFf;
        turretTurnFfDecayOverride = turnFfDecay;
        turretVisualDeadbandOverride = visDeadband;
        turretVisualMaxPowerOverride = visMaxPower;
        turretLimelightThresholdOverride = llThreshold;
    }
    
    /**
     * Reset visual tracking state (call when switching from visual to position tracking)
     */
    public void resetVisualTrackingState() {
        lastVisualError = 0.0;
    }
    
    /**
     * Get the smoothed turn feedforward value (for telemetry)
     */
    public double getSmoothedTurnFF() {
        return smoothedTurnFF;
    }
    
    /**
     * Update turn feedforward with smoothing/decay.
     * Call once per loop BEFORE turret control.
     * 
     * @param turnInput Raw turn input from gamepad (-1 to 1)
     * @param deltaTimeSec Time since last call in seconds
     */
    public void updateTurnFF(double turnInput, double deltaTimeSec) {
        double targetFF = turnInput * getEffectiveTurnFf();
        double decay = getEffectiveTurnFfDecay();
        
        if (decay <= 0) {
            smoothedTurnFF = targetFF;  // Instant response if decay disabled
        } else {
            double maxChange = decay * deltaTimeSec;
            double ffError = targetFF - smoothedTurnFF;
            if (Math.abs(ffError) <= maxChange) {
                smoothedTurnFF = targetFF;
            } else {
                smoothedTurnFF += Math.signum(ffError) * maxChange;
            }
        }
    }
    
    /**
     * Combined turret control method - handles both visual and position tracking.
     * This is the main method to call from OpModes.
     * 
     * @param allowVisualTracking True if visual tracking should be used when target visible
     * @param visualError The current tx error (targetTxOffset - tx)
     * @param targetTicks Target position in ticks (from odometry calculation)
     * @return TurretControlResult containing mode used and power applied
     */
    public TurretControlResult updateTurretControl(boolean allowVisualTracking, double visualError, double targetTicks) {
        double currentTicks = turretMotor.getCurrentPosition();
        double turretPower = 0;
        double pTerm = 0, dTerm = 0;
        boolean usingVisual = false;
        
        if (allowVisualTracking) {
            usingVisual = true;
            double derivative = visualError - lastVisualError;
            lastVisualError = visualError;
            
            if (Math.abs(visualError) < getEffectiveVisualDeadband()) {
                turretPower = 0;
            } else {
                pTerm = visualError * getEffectiveVisualKp();
                dTerm = derivative * getEffectiveVisualKd();
                double kfTerm = Math.signum(visualError) * getEffectiveVisualKf();
                turretPower = pTerm + dTerm + kfTerm + smoothedTurnFF;
                turretPower = Math.max(-getEffectiveVisualMaxPower(), Math.min(getEffectiveVisualMaxPower(), turretPower));
            }
        } else {
            lastVisualError = 0;
            double errorTicks = targetTicks - currentTicks;
            pTerm = errorTicks * getEffectiveTurretKp();
            turretPower = pTerm + smoothedTurnFF;
            turretPower = Math.max(-1.0, Math.min(1.0, turretPower));
        }
        
        turretMotor.setPower(turretPower);
        return new TurretControlResult(usingVisual, turretPower, pTerm, dTerm, smoothedTurnFF, currentTicks);
    }
    
    /**
     * Result object for turret control - contains useful telemetry data
     */
    public static class TurretControlResult {
        public final boolean usingVisualTracking;
        public final double power;
        public final double pTerm;
        public final double dTerm;
        public final double ffTerm;
        public final double currentTicks;
        
        public TurretControlResult(boolean usingVisual, double power, double pTerm, double dTerm, double ffTerm, double currentTicks) {
            this.usingVisualTracking = usingVisual;
            this.power = power;
            this.pTerm = pTerm;
            this.dTerm = dTerm;
            this.ffTerm = ffTerm;
            this.currentTicks = currentTicks;
        }
    }

    public void pointTurretVisualSimple(boolean isRedAlliance, double turnInput) {
        double tx = getLimelightTx(isRedAlliance);
        double error = targetTxOffset - tx;
        lastVisualError = error;
        if(Math.abs(error) < 1.5) {
            turretMotor.setPower(0.0);
            return;
        }

        double pTerm = error * getEffectiveVisualKp();
        double ffTerm = turnInput * getEffectiveTurnFf();
        double power = Math.max(-0.75, Math.min(0.75, pTerm + ffTerm));
        turretMotor.setPower(power);
    }
    public void pointTurretVisual(boolean isRedAlliance, double turnInput) {
         double tx = getLimelightTx(isRedAlliance);
         double error = targetTxOffset - tx;
         double derivative = error - lastVisualError;
         lastVisualError = error;
         if(Math.abs(error) < 1.5) {
             turretMotor.setPower(0.0);
             return;
         }
         
         double pTerm = error * getEffectiveVisualKp();
         double dTerm = derivative * getEffectiveVisualKd();
         double ffTerm = turnInput * getEffectiveTurnFf();
         double kfTerm = Math.signum(error) * getEffectiveVisualKf();
         double power = Math.max(-0.75, Math.min(0.75, pTerm + dTerm + ffTerm + kfTerm));
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

    public void stopShooter() {
        shooterMotor.setPower(0);
        lastShooterPower = 0.0;
    }

    /**
     * Set shooter motor power directly.
     */
    public void setShooterPower(double power) {
        shooterMotor.setPower(power);
        lastShooterPower = power;
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
        return getPowerConsumption() > 45;
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
        double currentAmps = intakeMotor.getCurrent(CurrentUnit.AMPS);
        double voltage = voltageSensor.getVoltage();
        return currentAmps * voltage;
    }

    // ========== LIMELIGHT FUNCTIONS ==========

    /**
     * Start the background limelight update thread.
     * Call this once during init after setting alliance.
     */
    public void startLimelightThread(boolean isRedAlliance) {
        isRedAllianceForLimelight = isRedAlliance;
        limelightThreadRunning = true;
        
        limelightThread = new Thread(() -> {
            while (limelightThreadRunning) {
                try {
                    updateLimelightDataInternal();
                } catch (Exception e) {
                    // Log error but don't crash - just invalidate cache
                    cachedHasTarget = false;
                    cachedTagId = -1;
                }
                try {
                    Thread.sleep(10); // Sleep to prevent CPU thrashing and reduce USB contention
                } catch (InterruptedException e) {
                    break;
                }
            }
        });
        limelightThread.setDaemon(true);
        limelightThread.setPriority(Thread.MIN_PRIORITY); // Lower priority than main loop
        limelightThread.start();
    }

    /**
     * Stop the background limelight thread.
     * Call this in stop() or when OpMode ends.
     */
    public void stopLimelightThread() {
        limelightThreadRunning = false;
        if (limelightThread != null) {
            limelightThread.interrupt();
            try {
                limelightThread.join(100);
            } catch (InterruptedException e) {
                // Ignore
            }
            limelightThread = null;
        }
    }

    /**
     * Internal method called by background thread to update limelight data.
     * Has robust exception handling to prevent thread crashes.
     */
    private void updateLimelightDataInternal() {
        LLResult result;
        try {
            result = limelight.getLatestResult();
        } catch (Exception e) {
            // USB/communication error - invalidate cache and return
            cachedHasTarget = false;
            cachedTagId = -1;
            return;
        }
        
        if (result == null || !result.isValid()) {
            cachedHasTarget = false;
            cachedTagId = -1;
            return;
        }
        
        try {
            int targetTagId = isRedAllianceForLimelight ? 24 : 20;
            
            // Process in local variables first
            double newTx = 0.0, newTy = 0.0, newTa = 0.0;
            int newTagId = -1;
            double newDistance = 0.0;
            
            LLResultTypes.FiducialResult goalTag = null;
            java.util.List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null) {
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if ((int) fiducial.getFiducialId() == targetTagId) {
                        goalTag = fiducial;
                        break;
                    }
                }
            }
            
            if (goalTag != null) {
                newTagId = (int) goalTag.getFiducialId();
                newTx = goalTag.getTargetXDegrees();
                newTy = goalTag.getTargetYDegrees();
                newTa = goalTag.getTargetArea();
                
                org.firstinspires.ftc.robotcore.external.navigation.Pose3D cameraPose = goalTag.getTargetPoseCameraSpace();
                if (cameraPose != null) {
                    double x = cameraPose.getPosition().x;
                    double y = cameraPose.getPosition().y;
                    double z = cameraPose.getPosition().z;
                    newDistance = Math.sqrt(x * x + y * y + z * z);
                }
            } else {
                // No correct tag found - use primary result for ty (shooter speed) only
                newTy = result.getTy();
                newTa = result.getTa();
            }
            
            // Track if limelight's internal timestamp changed (most reliable stale detection)
            double newTimestamp = result.getTimestamp();
            if (Math.abs(newTimestamp - lastLimelightTimestamp) > 0.001) {
                lastTimestampChangeTime = System.currentTimeMillis();
                lastLimelightTimestamp = newTimestamp;
            }
            
            // Update cached values (volatile writes are atomic for primitives)
            cachedLimelightResult = result;
            cachedTx = newTx;
            cachedTy = newTy;
            cachedTa = newTa;
            cachedTagId = newTagId;
            cachedTagDistance = newDistance;
            cachedHasTarget = true;
            lastLimelightUpdateTime = System.currentTimeMillis();
        } catch (Exception e) {
            // Processing error - invalidate cache
            cachedHasTarget = false;
            cachedTagId = -1;
        }
    }

    /**
     * Get time since last limelight update in ms (for staleness detection)
     */
    public long getLimelightAge() {
        return System.currentTimeMillis() - lastLimelightUpdateTime;
    }
    
    /**
     * Check if limelight frame is stale (internal timestamp unchanged for >150ms).
     * This detects frozen frames even when the SDK keeps returning "valid" results.
     */
    public boolean isLimelightFrameStale() {
        if (!cachedHasTarget || cachedTagId == -1) {
            return false;  // No target, so not "stale" - just no target
        }
        long timeSinceChange = System.currentTimeMillis() - lastTimestampChangeTime;
        return timeSinceChange > 150;  // 150ms threshold
    }
    
    /**
     * Get time since limelight timestamp actually changed (for debugging)
     */
    public long getFrameAge() {
        return System.currentTimeMillis() - lastTimestampChangeTime;
    }
    
    /**
     * Get the limelight's internal timestamp (for debugging)
     */
    public double getLimelightTimestamp() {
        return lastLimelightTimestamp;
    }

    /**
     * Synchronous limelight update - kept for backward compatibility.
     * Prefer using startLimelightThread() for better loop times.
     */
    public void updateLimelightData(boolean isRedAlliance) {
        isRedAllianceForLimelight = isRedAlliance;
        updateLimelightDataInternal();
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
        if (isRedAlliance && cachedTagId == 24 ) {
            return cachedTx;
        }else if(!isRedAlliance && cachedTagId == 20){
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

    // ========== INTAKE CONTROL ==========

    // Threshold for CR servo activation (motor can run at low power for holding, but servos shouldn't)
    private static final double SERVO_POWER_THRESHOLD = 0.5;

    /**
     * Run the intake system (motor + CR servos).
     * CR servos are coaxial with the motor, so they always run together.
     * Servos face opposite directions, so one runs positive and one negative.
     */
    public void runIntakeSystem(double power) {
        intakeMotor.setPower(power);
        // CR servos only run at higher power levels (not during low-power holding)
        if (Math.abs(power) >= SERVO_POWER_THRESHOLD) {
            intakeServoL.setPower(power);
            intakeServoR.setPower(-power);
        } else {
            intakeServoL.setPower(0);
            intakeServoR.setPower(0);
        }
    }

    public void stopIntakeSystem() {
        intakeMotor.setPower(0);
        intakeServoL.setPower(0);
        intakeServoR.setPower(0);
    }

    public void setIntakePower(double power) {
        boolean isRunning = Math.abs(power) > 0.05;
        if (isRunning && !transferWasRunning) {
            transferStartTime = System.currentTimeMillis();
        }
        transferWasRunning = isRunning;

        intakeMotor.setPower(power);
        // CR servos only run at higher power levels (not during low-power holding)
        if (Math.abs(power) >= SERVO_POWER_THRESHOLD) {
            intakeServoL.setPower(power);
            intakeServoR.setPower(-power);
        } else {
            intakeServoL.setPower(0);
            intakeServoR.setPower(0);
        }
    }

    public void setTransferPower(double power) {
        setIntakePower(power);  // Same system
    }
    
    /**
     * Run passive intake - motor only at low power, no servos.
     * Used for constant gentle intake when not actively feeding.
     * @param power Motor power (typically 0.2)
     */
    public void runPassiveIntake(double power) {
        intakeMotor.setPower(power);
        intakeServoL.setPower(0);
        intakeServoR.setPower(0);
    }

    /**
     * Check if intake has three balls (throttled to every 50ms to save loop time).
     * Uses cached result between checks to avoid expensive current/voltage reads.
     */
    public boolean hasThreeBalls() {
        if (!transferWasRunning) {
            cachedHasThreeBalls = false;
            return false;
        }

        long timeSinceStart = System.currentTimeMillis() - transferStartTime;
        if (timeSinceStart < RobotConstants.TRANSFER_STARTUP_IGNORE_TIME) {
            cachedHasThreeBalls = false;
            return false;
        }

        // Throttle expensive current/voltage reads to every 50ms
        long currentTime = System.currentTimeMillis();
        if (currentTime - lastThreeBallCheckTime >= THREE_BALL_CHECK_INTERVAL_MS) {
            lastThreeBallCheckTime = currentTime;
            double currentAmps = intakeMotor.getCurrent(CurrentUnit.AMPS);
            double voltage = voltageSensor.getVoltage();
            double power = currentAmps * voltage;
            cachedHasThreeBalls = power > RobotConstants.THREE_BALL_POWER_THRESHOLD;
        }
        
        return cachedHasThreeBalls;
    }

    public double getIntakeCurrent() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
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

    // ========== CLIMBER CONTROL ==========

    public void setClimberDown() {
        climberServo.setPosition(RobotConstants.getClimberDown());
    }

    public void setClimberUp() {
        climberServo.setPosition(RobotConstants.getClimberUp());
    }

    // ========== LED CONTROL ==========
    
    /**
     * Update LED color based on robot state.
     * 
     * LED KEY:
     * - WHITE:  Actively firing (latched and feeding)
     * - BLUE:   Ready to fire (shooter ready + turret on target + stationary)
     * - GREEN:  Shooter spinning + turret on target (waiting for speed)
     * - YELLOW: Visual tracking active (correct AprilTag detected)
     * - ORANGE: Shooter spinning but turret not aligned
     * - PURPLE: Intake full (3 balls detected)
     * - RED:    Idle / not ready
     */
    public void updateLightServo(boolean shooterRunning, boolean shooterReady, boolean isStationary,
                                  boolean shootingLatched, boolean hasCorrectTarget, boolean turretOnTarget,
                                  boolean usingVisualTracking, boolean hasThreeBalls) {
        double targetColor;

        if (shootingLatched) {
            // Actively firing
            targetColor = RobotConstants.LIGHT_WHITE;
        } else if (shooterReady && turretOnTarget && isStationary) {
            // Ready to fire - all conditions met
            targetColor = RobotConstants.LIGHT_BLUE;
        } else if (shooterRunning && turretOnTarget) {
            // Turret aligned, waiting for shooter speed
            targetColor = RobotConstants.LIGHT_GREEN;
        } else if (usingVisualTracking) {
            // Visual tracking active
            targetColor = RobotConstants.LIGHT_YELLOW;
        } else if (shooterRunning) {
            // Shooter spinning but turret not aligned
            targetColor = RobotConstants.LIGHT_ORANGE;
        } else if (hasThreeBalls) {
            // Intake full
            targetColor = RobotConstants.LIGHT_PURPLE;
        } else {
            // Idle
            targetColor = RobotConstants.LIGHT_RED;
        }

        lightServo.setPosition(targetColor);
    }

    public void setLightOff() {
        lightServo.setPosition(RobotConstants.LIGHT_OFF);
    }

    // ========== STOP ALL ==========

    public void stopAll() {
        stopLimelightThread();
        stopShooter();
        stopIntakeSystem();
        stopTurret();
    }
}
