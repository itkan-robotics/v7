package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Shooter {
    DcMotorEx shooterMotor;  // Single shooter motor
    DcMotor intakeMotor;
    DcMotor transferMotor;
    Servo blockerServo;
    Servo turretServo;  // Turret servo for aiming
    Servo indexingServo;  // Indexing servo for ball positioning
    
    // Constants for shooter subsystem
    public static final double SHOOTER_MAX_POWER = 1.0;
    public static final double SHOOTER_MIN_POWER = 0.0;
    public static final double SHOOTER_HOLD_POWER = 0.5;
    public static final double VELOCITY_TOLERANCE = 100.0; // ticks per second tolerance for bang-bang controller

    // Shooter velocity constants
    public static final double MIN_SHOOTER_VELOCITY = 1250.0;
    public static final double MAX_SHOOTER_VELOCITY = 2000.0;
    public static final double DEFAULT_TARGET_SHOOTER_VELOCITY = 1350.0;
    public static final double MIN_SHOOTER_VELOCITY_FOR_FEED = 1250.0;

    // Servo position constants
    public static final double BLOCKER_BLOCK_POSITION = 0.35;
    public static final double BLOCKER_UNBLOCK_POSITION = 0.65;
    
    // Indexer servo positions
    public static final double INDEXER_INDEXED = 0.45;   // Indexed position (ball ready)
    public static final double INDEXER_MIDDLE = 0.925;   // Middle/default position

    // Intake and transfer power constants
    public static final double INTAKE_POWER = 1.0;
    public static final double TRANSFER_POWER = 1.1;
    public static final double TRANSFER_POWER_FAR = 0.75;  // Reduced transfer speed when far away
    public static final double INTAKE_HOLD_POWER = 0.15; // Small power to keep balls inside during hold
    
    // Turret constants
    public static final double TURRET_CENTER_POSITION = 0.51;       // 0 degrees (center)
    public static final double TURRET_90_POSITION = 0.7633;         // 90 degrees (right)
    public static final double TURRET_NEG90_POSITION = 0.2561;      // -90 degrees (left)
    public static final double TURRET_MIN_ANGLE = -90.0;            // Minimum turret angle (left limit)
    public static final double TURRET_MAX_ANGLE = 180.0;             // Maximum turret angle (right limit)
    public static final double TURRET_DEGREES_PER_SERVO_UNIT = 355.0;  // Degrees of turret rotation per full servo travel
    public static final double TURRET_HOME_POSITION = 0.51;            // Center position (0 degrees turret angle)
    
    // Limelight auto-align settings for turret
    public static final double LIMELIGHT_KP = 0.02;  // Proportional gain for alignment
    public static final double LIMELIGHT_TOLERANCE = 2.0;  // Degrees tolerance for alignment
    public static final double SHOOTER_READY_ALIGNMENT_TOLERANCE_CLOSE = 5.0;  // Degrees tolerance when close
    public static final double SHOOTER_READY_ALIGNMENT_TOLERANCE_FAR = 2.0;    // Degrees tolerance when far
    public static final double APRILTAG_AREA_CLOSE_THRESHOLD = 0.5;  // Area threshold for close vs far distance
    public static final double APRILTAG_GOAL_OFFSET_MM = 508.0;  // 20 inches behind tag in millimeters

    // Current thresholds for detecting full intake (in Amperes)
    public static final double INTAKE_CURRENT_THRESHOLD = 2.0; // Adjust based on testing

    // Velocity thresholds for detecting full intake (in ticks per second)
    // When intake is full, motors will slow down or stall
    public static final double INTAKE_VELOCITY_THRESHOLD = 50.0; // Adjust based on testing
    public static final double INTAKE_STALL_VELOCITY = 10.0; // Velocity below which motor is considered stalled

    // Power consumption thresholds (as percentage of max power)
    public static final double INTAKE_POWER_THRESHOLD = 0.8; // 80% of max power indicates high load

    // Intake states
    public enum IntakeState {
        IDLE,           // No power, motors off
        HOLD,           // Small power to keep balls inside
        INTAKING,       // Intaking (negative power)
        OUTTAKING       // Outtaking/reversing (positive power)
    }

    private IntakeState currentIntakeState = IntakeState.IDLE;
    private boolean isBlockedState = false;

    public Shooter(HardwareMap hardwareMap) {
        // Initialize shooter motor (single motor, from HardwareConfigAuto)
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Initialize intake and transfer motors (from HardwareConfigAuto)
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer_motor");
        
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize blocker servo (from HardwareConfigAuto)
        blockerServo = hardwareMap.get(Servo.class, "blocker_servo");
        blockerServo.setPosition(BLOCKER_BLOCK_POSITION);
        isBlockedState = true;
        
        // Initialize turret servo
        turretServo = hardwareMap.get(Servo.class, "turret_servo");
        turretServo.setPosition(TURRET_HOME_POSITION);
        
        // Initialize indexing servo
        indexingServo = hardwareMap.get(Servo.class, "indexing_servo");
        indexingServo.setPosition(INDEXER_MIDDLE);
    }

    /**
     * Set the intake state (controls both intake and transfer motors)
     * @param state The intake state to set
     */
    public void setIntakeState(IntakeState state) {
        currentIntakeState = state;
        double power = 0.0;
        
        switch (state) {
            case INTAKING:
                power = INTAKE_POWER; // Positive for intaking (pulls balls in)
                break;
            case OUTTAKING:
                power = -INTAKE_POWER; // Negative for outtaking (pushes balls out)
                break;
            case HOLD:
                power = INTAKE_HOLD_POWER; // Small positive power to keep balls inside
                break;
            case IDLE:
            default:
                power = 0.0; // No power, motors off
                break;
        }
        
        intakeMotor.setPower(power);
        transferMotor.setPower(power);
    }
    
    /**
     * Run intake and transfer motors at specified power (for compatibility)
     * @param power Power level (-1.0 to 1.0, positive = intaking)
     */
    public void runIntakeSystem(double power) {
        intakeMotor.setPower(power);
        transferMotor.setPower(power);
        if (power > 0) {
            currentIntakeState = IntakeState.INTAKING;
        } else if (power < 0) {
            currentIntakeState = IntakeState.OUTTAKING;
        } else {
            currentIntakeState = IntakeState.IDLE;
        }
    }
    
    /**
     * Stop intake and transfer motors
     */
    public void stopIntakeSystem() {
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        currentIntakeState = IntakeState.IDLE;
    }

    /**
     * Get the current intake state
     * @return Current intake state
     */
    public IntakeState getIntakeState() {
        return currentIntakeState;
    }

    /**
     * Set the power of the intake motor directly (legacy method)
     * @param power Power value from -1.0 to 1.0
     */
    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
        transferMotor.setPower(power); // Transfer follows intake
    }

    /**
     * Set the power of the transfer motor directly (legacy method)
     * @param power Power value from -1.0 to 1.0
     */
    public void setTransferPower(double power) {
        transferMotor.setPower(power);
    }

    /**
     * Set shooter motor power
     * @param power Power value from -1.0 to 1.0
     */
    public void setShooterPower(double power) {
        shooterMotor.setPower(power);
    }

    /**
     * Get the current velocity of the shooter motor
     * @return Velocity in ticks per second
     */
    public double getShooterVelocity() {
        return Math.abs(shooterMotor.getVelocity());
    }

    /**
     * Block the shooter by setting the servo to blocked position
     */
    public void blockShooter() {
        blockerServo.setPosition(BLOCKER_BLOCK_POSITION);
        isBlockedState = true;
    }

    /**
     * Unblock the shooter by setting the servo to unblocked position
     */
    public void unblockShooter() {
        blockerServo.setPosition(BLOCKER_UNBLOCK_POSITION);
        isBlockedState = false;
    }
    
    /**
     * Set blocker servo position directly
     * @param position Servo position (0.0 to 1.0)
     */
    public void setBlockerPosition(double position) {
        blockerServo.setPosition(position);
        isBlockedState = (position == BLOCKER_BLOCK_POSITION);
    }
    
    /**
     * Set turret to a specific angle
     * @param turretAngle Desired turret angle in degrees (0 = center, positive = right, negative = left)
     */
    public void setTurretAngle(double turretAngle) {
        // Clamp turret angle to valid range
        double clampedAngle = turretAngle;
        if (clampedAngle < TURRET_MIN_ANGLE) {
            clampedAngle = TURRET_MIN_ANGLE;
        } else if (clampedAngle > TURRET_MAX_ANGLE) {
            clampedAngle = TURRET_MAX_ANGLE;
        }
        
        // Convert angle to servo position
        double servoPosition = TURRET_CENTER_POSITION + (clampedAngle / TURRET_DEGREES_PER_SERVO_UNIT);
        
        // Clamp to valid servo range
        if (servoPosition < 0.0) {
            servoPosition = 0.0;
        } else if (servoPosition > 1.0) {
            servoPosition = 1.0;
        }
        
        turretServo.setPosition(servoPosition);
    }
    
    /**
     * Set turret to home/center position (0 degrees)
     */
    public void setTurretHome() {
        turretServo.setPosition(TURRET_HOME_POSITION);
    }
    
    /**
     * Get current turret servo position
     * @return Current servo position (0.0 to 1.0)
     */
    public double getTurretServoPosition() {
        return turretServo.getPosition();
    }
    
    /**
     * Calculate current turret angle from servo position
     * @return Estimated turret angle in degrees
     */
    public double getTurretAngle() {
        double servoPosition = turretServo.getPosition();
        double turretAngle = (servoPosition - TURRET_CENTER_POSITION) * TURRET_DEGREES_PER_SERVO_UNIT;
        return turretAngle;
    }
    
    /**
     * Auto-align turret to target using Limelight tx value
     * Only aligns to AprilTag IDs 20 or 24
     * @param limelight Limelight subsystem instance
     * @return True if alignment was performed, false if no valid target
     */
    public boolean limelightTurretAutoAlign(Limelight limelight) {
        if (!limelight.hasTarget()) {
            return false;
        }
        
        int tagId = limelight.getAprilTagId();
        if (tagId != 20 && tagId != 24) {
            return false;
        }
        
        double tx = limelight.getTx();
        double targetOffset = calculateTargetOffset(limelight, tagId);
        double error = tx - targetOffset;
        
        if (Math.abs(error) > LIMELIGHT_TOLERANCE) {
            double currentTurretAngle = getTurretAngle();
            double turretAdjustment = -error * LIMELIGHT_KP;
            double newTurretAngle = currentTurretAngle + turretAdjustment;
            setTurretAngle(newTurretAngle);
            return true;
        }
        
        return false;
    }
    
    /**
     * Calculate target TX angle to aim at the goal position behind the AprilTag
     * @param limelight Limelight subsystem instance
     * @param tagId The detected AprilTag ID (20 or 24)
     * @return Target tx offset in degrees
     */
    private double calculateTargetOffset(Limelight limelight, int tagId) {
        com.qualcomm.hardware.limelightvision.LLResult result = limelight.getLatestResult();
        
        if (result == null || !result.isValid()) {
            return 0.0;
        }
        
        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
            return 0.0;
        }
        
        com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fiducial = result.getFiducialResults().get(0);
        org.firstinspires.ftc.robotcore.external.navigation.Pose3D tagPose = fiducial.getTargetPoseCameraSpace();
        
        if (tagPose == null) {
            return 0.0;
        }
        
        double tagX = tagPose.getPosition().x;
        double tagZ = tagPose.getPosition().z;
        double tagYaw = tagPose.getOrientation().getYaw(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);
        
        double goalDistanceBehind = APRILTAG_GOAL_OFFSET_MM;
        double goalX = tagX + goalDistanceBehind * Math.sin(tagYaw);
        double goalZ = tagZ + goalDistanceBehind * Math.cos(tagYaw);
        
        double targetTx = Math.toDegrees(Math.atan2(goalX, goalZ));
        return targetTx;
    }
    
    /**
     * Check if shooter is at target speed and aligned
     * @param limelight Limelight subsystem instance
     * @param targetVelocity Target velocity in ticks per second
     * @return True if shooter is within tolerance of target speed AND aligned
     */
    public boolean isShooterReady(Limelight limelight, double targetVelocity) {
        double currentVelocity = getShooterVelocity();
        boolean speedReady = Math.abs(currentVelocity - targetVelocity) <= VELOCITY_TOLERANCE;
        
        boolean aligned = false;
        if (limelight.hasTarget()) {
            int tagId = limelight.getAprilTagId();
            if (tagId == 20 || tagId == 24) {
                double currentTx = limelight.getTx();
                double targetOffset = calculateTargetOffset(limelight, tagId);
                double error = Math.abs(currentTx - targetOffset);
                
                double area = limelight.getTa();
                double tolerance = (area >= APRILTAG_AREA_CLOSE_THRESHOLD) ? 
                                  SHOOTER_READY_ALIGNMENT_TOLERANCE_CLOSE : 
                                  SHOOTER_READY_ALIGNMENT_TOLERANCE_FAR;
                aligned = error < tolerance;
            }
        }
        
        return speedReady && aligned;
    }
    
    /**
     * Get transfer motor speed based on AprilTag area (distance)
     * @param limelight Limelight subsystem instance
     * @return Transfer motor power
     */
    public double getTransferSpeed(Limelight limelight) {
        double area = limelight.getTa();
        if (area >= APRILTAG_AREA_CLOSE_THRESHOLD) {
            return TRANSFER_POWER;
        } else {
            return TRANSFER_POWER_FAR;
        }
    }
    
    /**
     * Get power consumption of intake and transfer motors
     * @return Power consumption in Watts (estimated)
     */
    public double getPowerConsumption() {
        double intakeCurrent = getIntakeCurrent();
        double transferCurrent = getTransferCurrent();
        // Assuming 12V nominal battery voltage
        return (intakeCurrent + transferCurrent) * 12.0;
    }

    /**
     * Check if the shooter is currently blocked
     * @return true if blocked, false if unblocked
     */
    public boolean isBlocked() {
        return isBlockedState;
    }

    /**
     * Update the shooter with bang-bang velocity control
     * @param enable If true, enable shooter and control to target velocity. If false, turn off shooter.
     * @param targetVelocity Target velocity in ticks per second
     */
    public void updateShooter(boolean enable, double targetVelocity) {
        if (!enable) {
            setShooterPower(0.0);
            return;
        }

        double currentVelocity = getShooterVelocity();
        double power;

        if (currentVelocity < (targetVelocity - VELOCITY_TOLERANCE)) {
            // Below target velocity, apply max power
            power = SHOOTER_MAX_POWER;
        } else {
            // At or above target velocity, apply min power
            power = SHOOTER_MIN_POWER;
        }

        setShooterPower(power);
    }
    
    /**
     * Update the shooter with bang-bang velocity control using high/hold power
     * @param enable If true, enable shooter and control to target velocity. If false, turn off shooter.
     * @param targetVelocity Target velocity in ticks per second
     */
    public void updateShooterWithHold(boolean enable, double targetVelocity) {
        if (!enable) {
            setShooterPower(0.0);
            return;
        }

        double currentVelocity = getShooterVelocity();
        double power;

        if (currentVelocity > targetVelocity) {
            // At or above target velocity, apply hold power
            power = SHOOTER_HOLD_POWER;
        } else {
            // Below target velocity, apply max power
            power = SHOOTER_MAX_POWER;
        }

        setShooterPower(power);
    }

    /**
     * Get the current draw of the intake motor in Amperes
     * @return Current draw in Amperes, or 0.0 if unavailable
     */
    public double getIntakeCurrent() {
        if (intakeMotor instanceof DcMotorEx) {
            return ((DcMotorEx) intakeMotor).getCurrent(CurrentUnit.AMPS);
        }
        return 0.0;
    }

    /**
     * Get the current draw of the transfer motor in Amperes
     * @return Current draw in Amperes, or 0.0 if unavailable
     */
    public double getTransferCurrent() {
        if (transferMotor instanceof DcMotorEx) {
            return ((DcMotorEx) transferMotor).getCurrent(CurrentUnit.AMPS);
        }
        return 0.0;
    }

    /**
     * Get the velocity of the intake motor
     * @return Velocity in ticks per second
     */
    public double getIntakeVelocity() {
        if (intakeMotor instanceof DcMotorEx) {
            return Math.abs(((DcMotorEx) intakeMotor).getVelocity());
        }
        return 0.0;
    }

    /**
     * Get the velocity of the transfer motor
     * @return Velocity in ticks per second
     */
    public double getTransferVelocity() {
        if (transferMotor instanceof DcMotorEx) {
            return Math.abs(((DcMotorEx) transferMotor).getVelocity());
        }
        return 0.0;
    }

    /**
     * Get the average velocity of intake and transfer motors
     * @return Average velocity in ticks per second
     */
    public double getIntakeTransferVelocity() {
        return (getIntakeVelocity() + getTransferVelocity()) / 2.0;
    }

    /**
     * Check if the intake is full based on motor current draw
     * When the intake is full, motors will stall or draw more current
     * @return true if intake appears to be full based on current thresholds
     */
    public boolean isIntakeFullByCurrent() {
        double intakeCurrent = getIntakeCurrent();
        double transferCurrent = getTransferCurrent();

        // Intake is considered full if either motor exceeds its threshold
        return intakeCurrent >= INTAKE_CURRENT_THRESHOLD || transferCurrent >= INTAKE_CURRENT_THRESHOLD;
    }

    /**
     * Check if the intake is full based on motor velocity
     * When the intake is full, motors will slow down or stall
     * @return true if intake appears to be full based on velocity thresholds
     */
    public boolean isIntakeFullByVelocity() {
        double avgVelocity = getIntakeTransferVelocity();
        
        // If motors are running but velocity is very low, likely stalled (full)
        if (currentIntakeState == IntakeState.INTAKING && avgVelocity < INTAKE_STALL_VELOCITY) {
            return true;
        }
        
        // If velocity drops significantly below expected, intake is likely full
        return avgVelocity < INTAKE_VELOCITY_THRESHOLD && currentIntakeState == IntakeState.INTAKING;
    }

    /**
     * Check if the intake is full based on power consumption
     * When the intake is full, motors will draw more power
     * @return true if intake appears to be full based on power thresholds
     */
    public boolean isIntakeFullByPower() {
        // Calculate power consumption as percentage
        double intakeCurrent = getIntakeCurrent();
        double transferCurrent = getTransferCurrent();
        
        // Estimate power consumption (current * voltage, normalized)
        // Assuming 12V nominal, max current around 2-3A for typical motors
        double maxExpectedCurrent = 3.0; // Amperes
        double intakePowerRatio = Math.min(intakeCurrent / maxExpectedCurrent, 1.0);
        double transferPowerRatio = Math.min(transferCurrent / maxExpectedCurrent, 1.0);
        
        // If power consumption is high, intake is likely full
        return intakePowerRatio >= INTAKE_POWER_THRESHOLD || transferPowerRatio >= INTAKE_POWER_THRESHOLD;
    }

    /**
     * Check if the intake is full using multiple detection methods
     * Combines velocity, current, and power consumption checks
     * @return true if intake appears to be full based on any detection method
     */
    public boolean isIntakeFull() {
        return isIntakeFullByCurrent() || isIntakeFullByVelocity() || isIntakeFullByPower();
    }
    
    /**
     * Alias for isIntakeFull() for compatibility with GatePath
     * @return true if intake appears to be full
     */
    public boolean intakeFull() {
        return isIntakeFull();
    }
    
    /**
     * Move indexing servo to indexed position (ball is moved out of the way)
     */
    public void setIndexerIndexed() {
        indexingServo.setPosition(INDEXER_INDEXED);
    }
    
    /**
     * Move indexing servo to middle position (ball is ready to shoot)
     */
    public void setIndexerMiddle() {
        indexingServo.setPosition(INDEXER_MIDDLE);
    }
    
    /**
     * Get current indexer position
     * @return Current servo position
     */
    public double getIndexerPosition() {
        return indexingServo.getPosition();
    }
    
    /**
     * Check if indexer is at middle position
     * @return True if at middle position
     */
    public boolean isIndexerAtMiddle() {
        double currentPos = indexingServo.getPosition();
        return Math.abs(currentPos - INDEXER_MIDDLE) < 0.05;
    }
}
