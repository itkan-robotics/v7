package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Configurable
@TeleOp(name="MainTeleop1", group="Linear Opmode")
public class MainTeleOp extends LinearOpMode {

    // ========== TURRET TUNING VALUES (Configurable via Panels) ==========
    // Position-based turret Kp
    public static double turretKp = RobotConstants.TURRET_KP;  // 0.04
    
    // Visual tracking PID
    public static double turretVisualKp = RobotConstants.TURRET_VISUAL_KP;  // 0.006
    public static double turretVisualKd = RobotConstants.TURRET_VISUAL_KD;  // 0.006
    public static double turretVisualKf = RobotConstants.TURRET_VISUAL_KF;  // 0.15
    
    // Turn feedforward
    public static double turretTurnFf = RobotConstants.TURRET_TURN_FF;  // -0.9
    
    // Panels telemetry manager
    private TelemetryManager panelsTelemetry;

    // Subsystems
    private Drive drive;
    private Shooter shooter;

    // Robot selection (21171 or 19564)
    private int selectedRobot = RobotConstants.ROBOT_19564;

    // Alliance tracking
    private boolean isRedAlliance = true;

    // Shooting state
    private boolean SHOOTING = false;
    private boolean shootingLatched = false;

    // Manual RPM mode
    private boolean manualRPMMode = false;
    private double manualTargetTPS = 1350.0;
    private static final double TPS_INCREMENT = 50.0;

    // Climber toggle
    private boolean climberUp = false;
    private boolean lastBackButton = false;

    // Manual TPS tuning
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    // Odometry reset
    private boolean lastLeftStickButton = false;
    
    // Turret zero
    private boolean lastRightStickButton = false;

    // Feeding timing for transfer power ramp
    private long feedingStartTime = 0;
    private boolean wasFeeding = false;
    
    // Sensor update optimization
    private double cachedTurretError = 999;
    private static final double LIMELIGHT_UPDATE_ERROR_THRESHOLD = 100.0;
    
    // Stationary check for shooting (uses drive encoder velocity, not odometry)
    private static final double STATIONARY_VELOCITY_THRESHOLD = 100.0;  // ticks/sec from drive encoders

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);  // Optimized: 50ms interval saves ~5-10ms per loop
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // ==================== INITIALIZE HARDWARE FIRST ====================
        RobotConstants.setRobot(RobotConstants.ROBOT_21171);

        // Initialize subsystems
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);

        // Start turret zeroing during init
        shooter.startTurretZero();

        // ==================== ROBOT & ALLIANCE SELECT ====================
        while (!opModeIsActive() && !isStopRequested()) {
            // Update turret zeroing during init
            shooter.updateTurretZero();
            
            telemetry.addData("=== ROBOT SELECT ===", "");
            telemetry.addLine("Y = 21171  |  A = 19564");
            telemetry.addData("Robot", selectedRobot);
            telemetry.addLine("");
            telemetry.addData("=== ALLIANCE SELECT ===", "");
            telemetry.addLine("B = RED  |  X = BLUE");
            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("Target Tag", isRedAlliance ? 24 : 20);
            telemetry.addLine("");
            telemetry.addData("Pinpoint Status", drive.getPinpointStatus());
            telemetry.addData("Turret Zero", shooter.isTurretZeroComplete() ? "COMPLETE" : "IN PROGRESS...");
            telemetry.addLine("");
            telemetry.addData("=== TURRET TUNING (Panels) ===", "");
            telemetry.addData("Pos Kp", "%.4f", turretKp);
            telemetry.addData("Vis Kp", "%.4f", turretVisualKp);
            telemetry.addData("Vis Kd", "%.4f", turretVisualKd);
            telemetry.addData("Vis Kf", "%.4f", turretVisualKf);
            telemetry.addData("Turn FF", "%.2f", turretTurnFf);
            telemetry.addLine("");
            telemetry.addData("Status", "Press START when ready");
            telemetry.update();
            panelsTelemetry.update(telemetry);

            // Robot selection
            if (gamepad1.y) {
                selectedRobot = RobotConstants.ROBOT_21171;
            } else if (gamepad1.a) {
                selectedRobot = RobotConstants.ROBOT_19564;
            }

            // Alliance selection
            if (gamepad1.b) {
                isRedAlliance = true;
            } else if (gamepad1.x) {
                isRedAlliance = false;
            }
        }

        // Apply the selected robot's constants
        RobotConstants.setRobot(selectedRobot);

        // If robot changed from default, re-apply hardware settings
        if (selectedRobot != RobotConstants.ROBOT_21171) {
            drive.applyPinpointSettings();
            shooter.applyMotorSettings();
        }

        // Initialize servos and set alliance
        shooter.initServos();
        drive.setAlliance(isRedAlliance);

        // Get goal position for turret tracking
        double goalX = isRedAlliance ? RobotConstants.GOAL_RED_X : RobotConstants.GOAL_BLUE_X;
        double goalY = isRedAlliance ? RobotConstants.GOAL_RED_Y : RobotConstants.GOAL_BLUE_Y;

        waitForStart();

        while (opModeIsActive()) {
            // ==================== CACHE HARDWARE READS ====================
            // Cache drive motor velocities once per loop (saves ~6ms by avoiding redundant reads)
            drive.cacheDriveVelocities();
            
            // ==================== SENSORS ====================
            // Calculate turret error first (uses previous frame's cached values)
            double turretTargetAngle = drive.calculateTurretAngleToGoal(goalX, goalY);
            double turretTargetTicks = shooter.angleToTurretTicks(turretTargetAngle);
            double turretCurrentTicks = shooter.getTurretEncoderPos();
            cachedTurretError = Math.abs(turretTargetTicks - turretCurrentTicks);
            
            // Update limelight only when turret is close to target
            // Invalidate cache when not updating to prevent stale visual tracking
            boolean limelightUpdatedThisFrame = cachedTurretError < LIMELIGHT_UPDATE_ERROR_THRESHOLD;
            if (limelightUpdatedThisFrame) {
                shooter.updateLimelightData(isRedAlliance);
            } else {
                shooter.invalidateLimelightCache();
            }
            
            // Now read limelight values (fresh or invalidated)
            boolean hasTarget = shooter.hasLimelightTarget();
            int detectedTagId = shooter.getDetectedAprilTagId(isRedAlliance);
            double tx = shooter.getLimelightTx(isRedAlliance);
            double ty = shooter.getLimelightTy();
            
            // Determine if we have the correct alliance tag
            int expectedTagId = isRedAlliance ? 24 : 20;
            boolean hasCorrectTarget = hasTarget && (detectedTagId == expectedTagId);
            
            // Visual tracking only allowed when limelight is fresh AND has correct target
            boolean allowVisualTracking = limelightUpdatedThisFrame && hasCorrectTarget;
            
            // Only update odometry when NOT using visual tracking
            if (!hasCorrectTarget) {
                drive.updateOdometry();
            }

            // ==================== DRIVETRAIN ====================
            double driveInput = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;
            drive.mecanumDriveWithBraking(driveInput, strafe, rotate, 1.0);

            // ==================== TURRET ====================
            boolean rightStickButton = gamepad1.right_stick_button;
            if (rightStickButton && !lastRightStickButton) {
                shooter.resetTurretZeroState();
                shooter.startTurretZero();
            }
            lastRightStickButton = rightStickButton;
            
            // Calculate auto TX offset based on predicted field position
            double fieldX = drive.getPredictedX() / RobotConstants.INCHES_TO_MM;
            double fieldY = drive.getPredictedY() / RobotConstants.INCHES_TO_MM;
            double autoTxOffset = calculateTxOffset(fieldX, fieldY);
            shooter.setTargetTxOffset(autoTxOffset);
            
            // Track if we're actually using visual tracking this frame
            boolean usingVisualTracking = false;
            if (!shooter.isTurretZeroComplete()) {
                shooter.updateTurretZero();
            } else {
                // Apply Panels-tuned PID values to shooter
                shooter.setTurretPidOverrides(turretKp, turretVisualKp, turretVisualKd, turretVisualKf, turretTurnFf);
                
                shooter.pointTurretAtGoal(isRedAlliance, allowVisualTracking, turretTargetAngle, rotate);
                // Visual tracking is active if allowed AND correct target is detected
                usingVisualTracking = allowVisualTracking && hasCorrectTarget;
            }

            // ==================== SHOOTER ====================
            // Manual TPS adjustment (Dpad up/down)
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            if (dpadUp && !lastDpadUp) {
                manualRPMMode = true;
                manualTargetTPS += TPS_INCREMENT;
                if (manualTargetTPS > 2500) manualTargetTPS = 2500;
            }
            if (dpadDown && !lastDpadDown) {
                manualRPMMode = true;
                manualTargetTPS -= TPS_INCREMENT;
                if (manualTargetTPS < 1000) manualTargetTPS = 1000;
            }
            lastDpadUp = dpadUp;
            lastDpadDown = dpadDown;

            // Get current shooter state
            double currentTPS = shooter.getShooterTPS();
            double targetTPS = manualRPMMode ? manualTargetTPS : shooter.getTargetShooterTPS();
            boolean shooterReady = shooter.isShooterSpeedReady(targetTPS);

            // A button: spin up shooter without shooting
            if (gamepad1.a) {
                if (manualRPMMode) {
                    shooter.controlShooterManual(manualTargetTPS, true);
                } else {
                    shooter.controlShooter(true);
                }
            }

            // ==================== SHOOTING SEQUENCE ====================
            boolean leftBumper = gamepad1.left_bumper;
            boolean rightBumper = gamepad1.right_bumper;
            boolean shootButtonPressed = leftBumper || rightBumper;

            // Preset positions for bumper shots
            if (leftBumper) {
//                drive.setPositionOverride(72.0, 30.0);
                shooter.setDefaultTPSOverride(1750.0);
            } else if (rightBumper) {
//                drive.setPositionOverride(72.0, 90.0);
                shooter.setDefaultTPSOverride(1550.0);
            } else {
//                drive.clearPositionOverride();
                shooter.clearDefaultTPSOverride();
            }

            // Check if robot is stationary using cached drive velocities (no extra hardware reads)
            double driveEncoderVelocity = drive.getCachedDriveVelocity();
            boolean isStationary = driveEncoderVelocity < STATIONARY_VELOCITY_THRESHOLD;
            
            // Shooting latch logic - only latch when stationary
            if (!shootButtonPressed) {
                shootingLatched = false;
            } else if (!shootingLatched && shooterReady && isStationary) {
                shootingLatched = true;
            }

            // Feed balls when shooting
            boolean feeding = false;
            if (shootButtonPressed) {
                shooter.setBlocker(false);
            }

            if (shootButtonPressed && shootingLatched && isStationary) {
                if (!wasFeeding) {
                    feedingStartTime = System.currentTimeMillis();
                }
                shooter.setIntakePower(-1.0);

                // Reduce transfer power for far shots after initial burst
                double transferPower = -1.0;
                if (targetTPS > 1650) {
                    long feedingDuration = System.currentTimeMillis() - feedingStartTime;
                    if (feedingDuration >= 250) {
                        transferPower = -0.65;
                    }
                }
                shooter.setTransferPower(transferPower);
                feeding = true;
            }
            wasFeeding = feeding;

            // Control shooter motor
            if (shootButtonPressed) {
                if (manualRPMMode) {
                    shooter.controlShooterManual(manualTargetTPS, true, feeding);
                } else {
                    shooter.controlShooter(true, feeding);
                }
                SHOOTING = true;
            } else if (!gamepad1.a) {
                shooter.controlShooter(false);
                SHOOTING = false;
            }

            // ==================== INTAKE ====================
            // Left trigger: reverse/unjam
            if (gamepad1.left_trigger > 0.1) {
                shooter.runIntakeSystem(1.0);
                shooter.setBlocker(true);
                feeding = true;
            }

            // Right trigger: intake
            if (gamepad1.right_trigger > 0.1) {
                shooter.runIntakeSystem(-1.0);
                shooter.setBlocker(true);
                feeding = true;
            }

            // Default state when not feeding
            if (!feeding) {
                shooter.setIntakePower(-0.1);
                if (!shootButtonPressed) {
                    shooter.setBlocker(true);
                }
            }

            // ==================== LED ====================
            boolean hasThreeBalls = shooter.hasThreeBalls();
            boolean turretOnTarget = cachedTurretError < 50;  // Turret considered on-target if error < 50 ticks
            shooter.updateLightServo(
                SHOOTING, shooterReady, feeding,
                feeding && SHOOTING && shooterReady,
                hasCorrectTarget, turretOnTarget, usingVisualTracking, hasThreeBalls
            );

            // ==================== CLIMBER ====================
            boolean backButton = gamepad1.back;
            if (backButton && !lastBackButton) {
                climberUp = !climberUp;
                if (climberUp) {
                    shooter.setClimberUp();
                } else {
                    shooter.setClimberDown();
                }
            }
            lastBackButton = backButton;

            // ==================== ODOMETRY RESET ====================
            boolean leftStickButton = gamepad1.left_stick_button;
            if (leftStickButton && !lastLeftStickButton) {
                double fieldCenterX = RobotConstants.FIELD_CENTER_X_INCHES * RobotConstants.INCHES_TO_MM;
                double fieldCenterY = RobotConstants.FIELD_CENTER_Y_INCHES * RobotConstants.INCHES_TO_MM;
                drive.setOdometryPosition(fieldCenterX, fieldCenterY, -90);
            }
            lastLeftStickButton = leftStickButton;

            // ==================== TELEMETRY ====================
            telemetry.addData("Status", "%s | %s | %s", 
                RobotConstants.getCurrentRobot(),
                isRedAlliance ? "RED" : "BLUE",
                SHOOTING ? "SHOOTING" : "IDLE");
            
            telemetry.addData("Shooter", "%.0f/%.0f TPS %s %s %s", 
                currentTPS, targetTPS,
                shooterReady ? "[RDY]" : "",
                isStationary ? "[STOP]" : "[MOVING]",
                manualRPMMode ? "[MAN]" : "");
            
            String turretMode = shooter.isTurretZeroComplete() ? 
                (usingVisualTracking ? "[VIS]" : "[POS]") : "[ZEROING]";
            telemetry.addData("Turret", "%.0f/%.0f err:%.0f pwr:%.3f %s",
                turretCurrentTicks, turretTargetTicks, cachedTurretError,
                shooter.getTurretPower(), turretMode);
            
            telemetry.addData("Limelight", "%s %s %s tx:%.1f ty:%.1f tgt:%.1f",
                hasTarget ? "TARGET" : "---",
                hasCorrectTarget ? "[OK]" : "",
                limelightUpdatedThisFrame ? "" : "[STALE]",
                tx, ty, autoTxOffset);
            
            // Debug: show all detected tags and compare primary vs goal tag tx
            // If turret is tracking wrong tag, tx and pri_tx will be same when they should differ
            double visualError = shooter.getTargetTxOffset() - tx;
            telemetry.addData("Tags", "all:[%s] using:%d exp:%d",
                shooter.getAllDetectedTagIds(),
                detectedTagId,
                expectedTagId);
            telemetry.addData("TxDbg", "goal:%.1f pri:%.1f err:%.2f %s",
                tx,
                shooter.getPrimaryTx(),
                visualError,
                usingVisualTracking ? "VIS" : "POS");
            
            telemetry.addData("Odom", "X:%.1f Y:%.1f H:%.1f %s",
                drive.getOdometryX() / 25.4, 
                drive.getOdometryY() / 25.4,
                drive.getOdometryHeading(),
                hasCorrectTarget ? "[FROZEN]" : "");
            
            telemetry.addLine("");
            telemetry.addData("=== TURRET TUNING (Panels) ===", "");
            telemetry.addData("Pos Kp", "%.4f", turretKp);
            telemetry.addData("Vis Kp", "%.4f", turretVisualKp);
            telemetry.addData("Vis Kd", "%.4f", turretVisualKd);
            telemetry.addData("Vis Kf", "%.4f", turretVisualKf);
            telemetry.addData("Turn FF", "%.2f", turretTurnFf);

            telemetry.update();
            panelsTelemetry.update(telemetry);
        }
        // Stop everything
        drive.stopMotors();
        shooter.stopAll();
    }
    
    /**
     * Calculate the optimal TX offset based on field position.
     * Near goal (high Y): positive offset (8-9 degrees in center)
     * Mid-field: zero offset
     * Far from goal (low Y): negative offset (up to -2 degrees)
     * 
     * @param fieldX Robot X position in inches
     * @param fieldY Robot Y position in inches
     * @return Optimal TX offset in degrees
     */
    private double calculateTxOffset(double fieldX, double fieldY) {
        double txOffset = 0.0;

        if (fieldY >= 130) {
            // Near goal baseline (Y ~144-146)
            // Offset varies by X: around 8-9 for center, 0 for edges
            if (fieldX >= 50 && fieldX <= 100) {
                // Center area - interpolate between data points
                // FX:64 → 9.4, FX:89 → 8.4
                double t = (fieldX - 64) / (89 - 64);
                txOffset = 9.4 + t * (8.4 - 9.4);  // 9.4 to 8.4
            } else if (fieldX < 50) {
                // Left side - fade to 0
                double t = fieldX / 50.0;
                txOffset = t * 9.4;  // 0 to 9.4
            } else {
                // Right side (X > 100) - fade to 0
                double t = (fieldX - 100) / 30.0;
                t = Math.min(1.0, t);
                txOffset = 8.4 * (1.0 - t);  // 8.4 to 0
            }
        } else if (fieldY >= 80) {
            // Mid-field (Y ~84-99) - offset is 0
            txOffset = 0.0;
        } else if (fieldY >= 30) {
            // Far from goal (Y ~35-80) - negative offset
            // Interpolate from 0 at Y=80 to -2 at Y=35
            double t = (80 - fieldY) / (80 - 35);
            txOffset = -2.0 * t;
        } else {
            // Very far - use max negative
            txOffset = -2.0;
        }

        return txOffset;
    }
}
