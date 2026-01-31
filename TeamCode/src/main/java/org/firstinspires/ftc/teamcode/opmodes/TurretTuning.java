package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.TurretTuning.drive;
import static org.firstinspires.ftc.teamcode.opmodes.TurretTuning.shooter;
import static org.firstinspires.ftc.teamcode.opmodes.TurretTuning.turretMotor;
import static org.firstinspires.ftc.teamcode.opmodes.TurretTuning.telemetryM;
import static org.firstinspires.ftc.teamcode.opmodes.TurretTuning.isRedAlliance;
import static org.firstinspires.ftc.teamcode.opmodes.TurretTuning.goalX;
import static org.firstinspires.ftc.teamcode.opmodes.TurretTuning.goalY;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.List;

/**
 * Unified Turret Tuning OpMode with selectable modes:
 * - Odometry Test: Position-based tracking only (no Limelight)
 * - Visual Test: Limelight-based tracking only
 * - Combined Test: Full logic with odometry + visual switching
 */
@Configurable
@TeleOp(name = "Turret Tuning", group = "Tuning")
public class TurretTuning extends SelectableOpMode {

    // Shared hardware
    @IgnoreConfigurable
    public static Drive drive;
    @IgnoreConfigurable
    public static Shooter shooter;
    @IgnoreConfigurable
    public static DcMotorEx turretMotor;
    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    // Shared state
    @IgnoreConfigurable
    public static boolean isRedAlliance = true;
    @IgnoreConfigurable
    public static double goalX;
    @IgnoreConfigurable
    public static double goalY;

    // ========== TUNING PARAMETERS (Configurable via Panels) ==========
    // Position-based tracking
    public static double posKp = RobotConstants.TURRET_KP;

    // Visual tracking (tuned values now in RobotConstants)
    public static double visualKp = RobotConstants.TURRET_VISUAL_KP;
    public static double visualKd = RobotConstants.TURRET_VISUAL_KD;
    public static double visualKf = RobotConstants.TURRET_VISUAL_KF;
    public static double visualDeadband = RobotConstants.TURRET_VISUAL_DEADBAND;
    public static double visualMaxPower = RobotConstants.TURRET_VISUAL_MAX_POWER;
    public static double targetTxOffset = 0.0;

    // Turn feedforward (tuned values now in RobotConstants)
    public static double turnFF = RobotConstants.TURRET_TURN_FF;
    public static double turnFFDecay = RobotConstants.TURRET_TURN_FF_DECAY;

    // Combined mode threshold (tuned value now in RobotConstants)
    public static double limelightThreshold = RobotConstants.TURRET_LIMELIGHT_THRESHOLD;

    public TurretTuning() {
        super("Select Turret Test Mode", s -> {
            s.add("Odometry Test", OdometryTest::new);
            s.add("Visual Test", VisualTest::new);
            s.add("Combined Test", CombinedTest::new);
        });
    }

    @Override
    public void onSelect() {
        // Initialize hardware
        RobotConstants.setRobot(RobotConstants.ROBOT_21171);
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize odometry - must call update to populate cached values
        drive.updateOdometry();

        // Set alliance and goal
        drive.setAlliance(isRedAlliance);
        goalX = isRedAlliance ? RobotConstants.GOAL_RED_X : RobotConstants.GOAL_BLUE_X;
        goalY = isRedAlliance ? RobotConstants.GOAL_RED_Y : RobotConstants.GOAL_BLUE_Y;

        // Start turret zeroing
        shooter.startTurretZero();
    }

    @Override
    public void onLog(List<String> lines) {}

    // Helper to update alliance selection
    public static void updateAlliance(boolean red) {
        isRedAlliance = red;
        drive.setAlliance(isRedAlliance);
        goalX = isRedAlliance ? RobotConstants.GOAL_RED_X : RobotConstants.GOAL_BLUE_X;
        goalY = isRedAlliance ? RobotConstants.GOAL_RED_Y : RobotConstants.GOAL_BLUE_Y;
    }
}

// ==================== ODOMETRY TEST ====================
class OdometryTest extends OpMode {
    private boolean lastLeftStickButton = false;
    private boolean lastRightStickButton = false;
    private boolean turretZeroing = false;
    private long turretZeroStartTime = 0;
    private double smoothedTurnFF = 0.0;  // For turn FF ramp-down
    private long lastLoopTime = 0;

    @Override
    public void init() {
        shooter.startTurretZero();
    }

    @Override
    public void init_loop() {
        shooter.updateTurretZero();

        if (gamepad1.b) TurretTuning.updateAlliance(true);
        if (gamepad1.x) TurretTuning.updateAlliance(false);

        telemetryM.debug("=== ODOMETRY TURRET TEST ===");
        telemetryM.debug("Position-based tracking (NO Limelight)");
        telemetryM.debug("");
        telemetryM.debug("B = RED | X = BLUE");
        telemetryM.debug("Alliance: " + (isRedAlliance ? "RED" : "BLUE"));
        telemetryM.debug("Turret Zero: " + (shooter.isTurretZeroComplete() ? "COMPLETE" : "IN PROGRESS"));
        telemetryM.debug("");
        telemetryM.debug("posKp: " + TurretTuning.posKp);
        telemetryM.debug("turnFF: " + TurretTuning.turnFF);
        telemetryM.debug("");
        telemetryM.debug("LStick Btn = Reset Odom | RStick Btn = Zero Turret");
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        lastLoopTime = System.nanoTime();
    }

    @Override
    public void loop() {
        long loopStart = System.nanoTime();
        double deltaTimeSec = (loopStart - lastLoopTime) / 1_000_000_000.0;
        if (deltaTimeSec > 0.1) deltaTimeSec = 0.1;  // Cap at 100ms to handle first frame
        double loopTimeMs = (loopStart - lastLoopTime) / 1_000_000.0;
        lastLoopTime = loopStart;

        // Update odometry
        drive.updateOdometry();

        // Button handling
        boolean leftStickButton = gamepad1.left_stick_button;
        if (leftStickButton && !lastLeftStickButton) {
            double fieldCenterX = RobotConstants.FIELD_CENTER_X_INCHES * RobotConstants.INCHES_TO_MM;
            double fieldCenterY = RobotConstants.FIELD_CENTER_Y_INCHES * RobotConstants.INCHES_TO_MM;
            drive.setOdometryPosition(fieldCenterX, fieldCenterY, -90);
        }
        lastLeftStickButton = leftStickButton;

        boolean rightStickButton = gamepad1.right_stick_button;
        if (rightStickButton && !lastRightStickButton && !turretZeroing) {
            turretZeroing = true;
            turretZeroStartTime = System.currentTimeMillis();
            shooter.resetTurretZeroState();
            shooter.startTurretZero();
        }
        lastRightStickButton = rightStickButton;

        // Drivetrain
        double driveInput = -gamepad1.left_stick_y;
        double strafeInput = gamepad1.left_stick_x;
        double turnInput = -gamepad1.right_stick_x;
        drive.mecanumDrive(driveInput, strafeInput, turnInput, 1.0);

        // Turn FF smoothing - ramp towards target value
        double targetFF = turnInput * TurretTuning.turnFF;
        if (TurretTuning.turnFFDecay <= 0) {
            smoothedTurnFF = targetFF;  // Instant response if decay disabled
        } else {
            double maxChange = TurretTuning.turnFFDecay * deltaTimeSec;
            double ffError = targetFF - smoothedTurnFF;
            if (Math.abs(ffError) <= maxChange) {
                smoothedTurnFF = targetFF;
            } else {
                smoothedTurnFF += Math.signum(ffError) * maxChange;
            }
        }

        // Turret control
        double targetAngle = drive.calculateTurretAngleToGoal(goalX, goalY);
        double targetTicks = shooter.angleToTurretTicks(targetAngle);
        double currentTicks = turretMotor.getCurrentPosition();
        double errorTicks = targetTicks - currentTicks;

        double turretPower = 0;
        if (turretZeroing || !shooter.isTurretZeroComplete()) {
            shooter.updateTurretZero();
            if (shooter.isTurretZeroComplete()) turretZeroing = false;
        } else {
            double pTerm = errorTicks * TurretTuning.posKp;
            turretPower = Math.max(-1.0, Math.min(1.0, pTerm + smoothedTurnFF));
            // Use safe power method with hardstop protection
            turretPower = shooter.setTurretPowerSafe(turretPower);
        }

        // Telemetry
        telemetryM.debug("=== ODOMETRY ===");
        telemetryM.debug("X: " + String.format("%.1f", drive.getOdometryX() / 25.4) + " in");
        telemetryM.debug("Y: " + String.format("%.1f", drive.getOdometryY() / 25.4) + " in");
        telemetryM.debug("H: " + String.format("%.1f", drive.getOdometryHeading()) + "°");
        telemetryM.debug("");
        telemetryM.debug("=== TURRET ===");
        telemetryM.debug("Target: " + String.format("%.1f", targetAngle) + "°");
        telemetryM.debug("Ticks: " + String.format("%.0f / %.0f", currentTicks, targetTicks));
        telemetryM.debug("Error: " + String.format("%.0f", errorTicks));
        telemetryM.debug("FF: raw=" + String.format("%.3f", targetFF) + " smooth=" + String.format("%.3f", smoothedTurnFF));
        telemetryM.debug("Power: " + String.format("%.3f", turretPower));
        telemetryM.debug("");
        telemetryM.debug("Loop: " + String.format("%.1f ms (%.0f Hz)", loopTimeMs, 1000.0/loopTimeMs));
        telemetryM.update(telemetry);
    }
}

// ==================== VISUAL TEST ====================
class VisualTest extends OpMode {
    private boolean lastRightStickButton = false;
    private boolean turretZeroing = false;
    private double lastVisualError = 0.0;
    private double smoothedTurnFF = 0.0;
    private long lastLoopTime = 0;

    @Override
    public void init() {
        shooter.startTurretZero();
    }

    @Override
    public void init_loop() {
        shooter.updateTurretZero();

        if (gamepad1.b) TurretTuning.updateAlliance(true);
        if (gamepad1.x) TurretTuning.updateAlliance(false);

        telemetryM.debug("=== VISUAL TURRET TEST ===");
        telemetryM.debug("Limelight-based tracking (PD control)");
        telemetryM.debug("");
        telemetryM.debug("B = RED | X = BLUE");
        telemetryM.debug("Alliance: " + (isRedAlliance ? "RED" : "BLUE"));
        telemetryM.debug("Target Tag: " + (isRedAlliance ? 24 : 20));
        telemetryM.debug("Turret Zero: " + (shooter.isTurretZeroComplete() ? "COMPLETE" : "IN PROGRESS"));
        telemetryM.debug("");
        telemetryM.debug("Kp: " + TurretTuning.visualKp + " | Kd: " + TurretTuning.visualKd);
        telemetryM.debug("Deadband: " + TurretTuning.visualDeadband + "°");
        telemetryM.debug("TurnFF: " + TurretTuning.turnFF + " | Decay: " + TurretTuning.turnFFDecay);
        telemetryM.debug("");
        telemetryM.debug("RStick Btn = Zero Turret");
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        lastLoopTime = System.nanoTime();
    }

    @Override
    public void loop() {
        long loopStart = System.nanoTime();
        double deltaTimeSec = (loopStart - lastLoopTime) / 1_000_000_000.0;
        if (deltaTimeSec > 0.1) deltaTimeSec = 0.1;
        double loopTimeMs = (loopStart - lastLoopTime) / 1_000_000.0;
        lastLoopTime = loopStart;

        // Limelight update
        shooter.updateLimelightData(isRedAlliance);
        double tx = shooter.getLimelightTx(isRedAlliance);
        double ty = shooter.getLimelightTy();
        int detectedTagId = shooter.getDetectedAprilTagId(isRedAlliance);
        boolean hasTarget = shooter.hasLimelightTarget();
        int expectedTagId = isRedAlliance ? 24 : 20;
        boolean hasCorrectTarget = hasTarget && (detectedTagId == expectedTagId);

        // Button handling
        boolean rightStickButton = gamepad1.right_stick_button;
        if (rightStickButton && !lastRightStickButton && !turretZeroing) {
            turretZeroing = true;
            shooter.resetTurretZeroState();
            shooter.startTurretZero();
        }
        lastRightStickButton = rightStickButton;

        // Drivetrain
        double driveInput = -gamepad1.left_stick_y;
        double strafeInput = gamepad1.left_stick_x;
        double turnInput = -gamepad1.right_stick_x;
        drive.mecanumDrive(driveInput, strafeInput, turnInput, 1.0);

        // Turn FF smoothing
        double targetFF = turnInput * TurretTuning.turnFF;
        if (TurretTuning.turnFFDecay <= 0) {
            smoothedTurnFF = targetFF;
        } else {
            double maxChange = TurretTuning.turnFFDecay * deltaTimeSec;
            double ffError = targetFF - smoothedTurnFF;
            if (Math.abs(ffError) <= maxChange) {
                smoothedTurnFF = targetFF;
            } else {
                smoothedTurnFF += Math.signum(ffError) * maxChange;
            }
        }

        // Turret control
        double turretPower = 0;
        double pTerm = 0, dTerm = 0;
        double visualError = TurretTuning.targetTxOffset - tx;

        if (turretZeroing || !shooter.isTurretZeroComplete()) {
            shooter.updateTurretZero();
            if (shooter.isTurretZeroComplete()) turretZeroing = false;
        } else if (hasCorrectTarget) {
            double derivative = visualError - lastVisualError;
            lastVisualError = visualError;

            if (Math.abs(visualError) < TurretTuning.visualDeadband) {
                turretPower = 0;
            } else {
                pTerm = visualError * TurretTuning.visualKp;
                dTerm = derivative * TurretTuning.visualKd;
                double kfTerm = Math.signum(visualError) * TurretTuning.visualKf;
                turretPower = pTerm + dTerm + kfTerm + smoothedTurnFF;
                turretPower = Math.max(-TurretTuning.visualMaxPower, Math.min(TurretTuning.visualMaxPower, turretPower));
            }
            // Use safe power method with hardstop protection
            turretPower = shooter.setTurretPowerSafe(turretPower);
        } else {
            shooter.setTurretPowerSafe(0);
            lastVisualError = 0;
        }
        double turretCurrentTicks = turretMotor.getCurrentPosition();

        // Telemetry
        telemetryM.debug("=== LIMELIGHT ===");
        telemetryM.debug("Target: " + (hasCorrectTarget ? "YES (" + detectedTagId + ")" : "NO"));
        telemetryM.debug("tx: " + String.format("%.2f", tx) + "° | ty: " + String.format("%.2f", ty) + "°");
        telemetryM.debug("");
        telemetryM.debug("=== TURRET ===");
        telemetryM.debug("Status: " + (hasCorrectTarget ? "TRACKING" : "NO TARGET"));
        telemetryM.debug("Error: " + String.format("%.2f", visualError) + "°");
        telemetryM.debug("P: " + String.format("%.4f", pTerm) + " | D: " + String.format("%.4f", dTerm));
        telemetryM.debug("FF: " + String.format("%.4f", smoothedTurnFF));
        telemetryM.debug("Power: " + String.format("%.3f", turretPower));
        telemetryM.debug("");
        telemetryM.debug("Loop: " + String.format("%.1f ms (%.0f Hz)", loopTimeMs, 1000.0/loopTimeMs));
        telemetryM.debug("current ticks of turret" + turretCurrentTicks);
        telemetryM.update(telemetry);
    }
}

// ==================== COMBINED TEST ====================
class CombinedTest extends OpMode {
    private boolean lastLeftStickButton = false;
    private boolean lastRightStickButton = false;
    private double lastVisualError = 0.0;
    private double cachedTurretError = 999;
    private double smoothedTurnFF = 0.0;  // For turn FF ramp-down
    private long lastLoopTime = 0;

    @Override
    public void init() {
        shooter.startTurretZero();
    }

    @Override
    public void init_loop() {
        shooter.updateTurretZero();

        if (gamepad1.b) TurretTuning.updateAlliance(true);
        if (gamepad1.x) TurretTuning.updateAlliance(false);

        telemetryM.debug("=== COMBINED TURRET TEST ===");
        telemetryM.debug("Full odometry + visual tracking logic");
        telemetryM.debug("");
        telemetryM.debug("B = RED | X = BLUE");
        telemetryM.debug("Alliance: " + (isRedAlliance ? "RED" : "BLUE"));
        telemetryM.debug("Target Tag: " + (isRedAlliance ? 24 : 20));
        telemetryM.debug("Turret Zero: " + (shooter.isTurretZeroComplete() ? "COMPLETE" : "IN PROGRESS"));
        telemetryM.debug("");
        telemetryM.debug("Pos Kp: " + TurretTuning.posKp);
        telemetryM.debug("Vis Kp: " + TurretTuning.visualKp + " | Kd: " + TurretTuning.visualKd);
        telemetryM.debug("Deadband: " + TurretTuning.visualDeadband + "° | LL Thresh: " + TurretTuning.limelightThreshold);
        telemetryM.debug("");
        telemetryM.debug("LStick Btn = Reset Odom | RStick Btn = Zero Turret");
        telemetryM.update(telemetry);
    }

    @Override
    public void start() {
        lastLoopTime = System.nanoTime();
        shooter.setTargetTxOffset(TurretTuning.targetTxOffset);
    }

    @Override
    public void loop() {
        long loopStart = System.nanoTime();
        double deltaTimeSec = (loopStart - lastLoopTime) / 1_000_000_000.0;
        if (deltaTimeSec > 0.1) deltaTimeSec = 0.1;  // Cap at 100ms to handle first frame
        double loopTimeMs = (loopStart - lastLoopTime) / 1_000_000.0;
        lastLoopTime = loopStart;

        // Cache drive velocities (for braking and stationary check)
        drive.cacheDriveVelocities();

        // Update limelight first to determine if we should freeze odometry
        // Calculate preliminary turret error using cached odometry values
        double prelimTurretAngle = drive.calculateTurretAngleToGoal(goalX, goalY);
        double prelimTurretTicks = shooter.angleToTurretTicks(prelimTurretAngle);
        double turretCurrentTicks = turretMotor.getCurrentPosition();
        cachedTurretError = Math.abs(prelimTurretTicks - turretCurrentTicks);

        // Conditional limelight update (only when turret is close to target)
        boolean limelightUpdated = cachedTurretError < TurretTuning.limelightThreshold;
        if (limelightUpdated) {
            shooter.updateLimelightData(isRedAlliance);
        } else {
            shooter.invalidateLimelightCache();
        }

        boolean hasTarget = shooter.hasLimelightTarget();
        int detectedTagId = shooter.getDetectedAprilTagId(isRedAlliance);
        double tx = shooter.getLimelightTx(isRedAlliance);
        int expectedTagId = isRedAlliance ? 24 : 20;
        boolean hasCorrectTarget = hasTarget && (detectedTagId == expectedTagId);
        boolean allowVisualTracking = limelightUpdated && hasCorrectTarget;

        // Update odometry BEFORE turret calculation (unless visual tracking is active)
        // This ensures turret uses fresh position data
        if (!hasCorrectTarget) {
            drive.updateOdometry();
        }

        // Now calculate final turret target with updated odometry
        double turretTargetAngle = drive.calculateTurretAngleToGoal(goalX, goalY);
        double turretTargetTicks = shooter.angleToTurretTicks(turretTargetAngle);
        cachedTurretError = Math.abs(turretTargetTicks - turretCurrentTicks);

        // Button handling
        boolean leftStickButton = gamepad1.left_stick_button;
        if (leftStickButton && !lastLeftStickButton) {
            double fieldCenterX = RobotConstants.FIELD_CENTER_X_INCHES * RobotConstants.INCHES_TO_MM;
            double fieldCenterY = RobotConstants.FIELD_CENTER_Y_INCHES * RobotConstants.INCHES_TO_MM;
            drive.setOdometryPosition(fieldCenterX, fieldCenterY, -90);
        }
        lastLeftStickButton = leftStickButton;

        boolean rightStickButton = gamepad1.right_stick_button;
        if (rightStickButton && !lastRightStickButton) {
            shooter.resetTurretZeroState();
            shooter.startTurretZero();
        }
        lastRightStickButton = rightStickButton;

        // Drivetrain
        double driveInput = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;
        drive.mecanumDriveWithBraking(driveInput, strafe, rotate, 1.0);

        // Turn FF smoothing - ramp towards target value
        double targetFF = rotate * TurretTuning.turnFF;
        if (TurretTuning.turnFFDecay <= 0) {
            smoothedTurnFF = targetFF;  // Instant response if decay disabled
        } else {
            double maxChange = TurretTuning.turnFFDecay * deltaTimeSec;
            double ffError = targetFF - smoothedTurnFF;
            if (Math.abs(ffError) <= maxChange) {
                smoothedTurnFF = targetFF;
            } else {
                smoothedTurnFF += Math.signum(ffError) * maxChange;
            }
        }

        // Turret control
        double turretPower = 0;
        double pTerm = 0, dTerm = 0, ffTerm = smoothedTurnFF;
        double visualError = TurretTuning.targetTxOffset - tx;
        boolean usingVisual = false;

        if (!shooter.isTurretZeroComplete()) {
            shooter.updateTurretZero();
        } else if (allowVisualTracking) {
            usingVisual = true;
            double derivative = visualError - lastVisualError;
            lastVisualError = visualError;

            if (Math.abs(visualError) < TurretTuning.visualDeadband) {
                turretPower = 0;
            } else {
                pTerm = visualError * TurretTuning.visualKp;
                dTerm = derivative * TurretTuning.visualKd;
                double kfTerm = Math.signum(visualError) * TurretTuning.visualKf;
                turretPower = pTerm + dTerm + kfTerm + ffTerm;
                turretPower = Math.max(-TurretTuning.visualMaxPower, Math.min(TurretTuning.visualMaxPower, turretPower));
            }
            // Use safe power method with hardstop protection
            turretPower = shooter.setTurretPowerSafe(turretPower);
        } else {
            lastVisualError = 0;
            double errorTicks = turretTargetTicks - turretCurrentTicks;
            pTerm = errorTicks * TurretTuning.posKp;
            turretPower = pTerm + ffTerm;
            turretPower = Math.max(-1.0, Math.min(1.0, turretPower));
            // Use safe power method with hardstop protection
            turretPower = shooter.setTurretPowerSafe(turretPower);
        }

        // Telemetry
        String mode = shooter.isTurretZeroComplete() ? (usingVisual ? "VISUAL" : "POSITION") : "ZEROING";
        telemetryM.debug("Mode: " + mode + " | Alliance: " + (isRedAlliance ? "RED" : "BLUE"));
        telemetryM.debug("");
        telemetryM.debug("=== TURRET ===");
        telemetryM.debug("Ticks: " + String.format("%.0f / %.0f", turretCurrentTicks, turretTargetTicks));
        telemetryM.debug("Pos Err: " + String.format("%.0f", cachedTurretError) + " | Vis Err: " + String.format("%.2f", visualError) + "°");
        telemetryM.debug("P: " + String.format("%.4f", pTerm) + " | D: " + String.format("%.4f", dTerm));
        telemetryM.debug("FF: raw=" + String.format("%.3f", rotate * TurretTuning.turnFF) + " smooth=" + String.format("%.3f", smoothedTurnFF));
        telemetryM.debug("Power: " + String.format("%.3f", turretPower));
        telemetryM.debug("");
        telemetryM.debug("=== LIMELIGHT ===");
        telemetryM.debug("Updated: " + (limelightUpdated ? "YES" : "NO (turret far)"));
        telemetryM.debug("Correct Tag: " + (hasCorrectTarget ? "YES (" + detectedTagId + ")" : "NO"));
        telemetryM.debug("tx: " + String.format("%.2f", tx) + "°");
        telemetryM.debug("");
        telemetryM.debug("=== ODOMETRY ===");
        telemetryM.debug("X: " + String.format("%.1f", drive.getOdometryX()/25.4) + " in");
        telemetryM.debug("Y: " + String.format("%.1f", drive.getOdometryY()/25.4) + " in");
        telemetryM.debug("H: " + String.format("%.1f", drive.getOdometryHeading()) + "° " + (hasCorrectTarget ? "[FROZEN]" : ""));
        telemetryM.debug("");
        telemetryM.debug("Loop: " + String.format("%.1f ms (%.0f Hz)", loopTimeMs, 1000.0/loopTimeMs));
        telemetryM.update(telemetry);
    }
}

