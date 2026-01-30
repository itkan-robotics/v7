package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Test TeleOp Mode
 * - Uses RUN_TO_POSITION for all turret control (instead of custom PID)
 * - Variable shooter velocity based on distance (ty-based lookup)
 * - Synchronous limelight updates (no multithreading)
 * - Simplified shooting logic for testing
 */
@TeleOp(name="TestTeleOp", group="Test")
public class TestTeleOp extends LinearOpMode {

    // Subsystems
    private Drive drive;
    private Shooter shooter;

    // Alliance tracking
    private boolean isRedAlliance = true;

    // Shooting state
    private boolean SHOOTING = false;
    private boolean shootingLatched = false;

    // Turret RUN_TO_POSITION power
    private static final double TURRET_POWER = 0.7;

    // Climber toggle
    private boolean climberUp = false;
    private boolean lastBackButton = false;

    // Odometry reset
    private boolean lastLeftStickButton = false;

    // Turret zero
    private boolean lastRightStickButton = false;

    // Feeding timing
    private long feedingStartTime = 0;
    private boolean wasFeeding = false;

    // Shooting timing
    private static final double STATIONARY_VELOCITY_THRESHOLD = 100.0;
    private static final double TURRET_FIRE_TOLERANCE = 50.0;
    private static final long MIN_SPINUP_TIME_MS = 200;
    private long shooterSpinupStartTime = 0;
    private boolean wasShootButtonPressed = false;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);

        // ==================== INITIALIZE HARDWARE ====================
        RobotConstants.setRobot(RobotConstants.ROBOT_21171);

        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);

        drive.updateOdometry();
        shooter.startTurretZero();

        // ==================== ALLIANCE SELECT ====================
        while (!opModeIsActive() && !isStopRequested()) {
            shooter.updateTurretZero();

            telemetry.addData("=== TEST TELEOP (21171 LUNAR) ===", "");
            telemetry.addData("Shooter TPS", "Variable (ty-based)");
            telemetry.addData("Turret Mode", "RUN_TO_POSITION");
            telemetry.addLine("");
            telemetry.addData("=== ALLIANCE SELECT ===", "");
            telemetry.addLine("B = RED  |  X = BLUE");
            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addLine("");
            telemetry.addData("Turret Zero", shooter.isTurretZeroComplete() ? "COMPLETE" : "IN PROGRESS...");
            telemetry.addData("Status", "Press START when ready");
            telemetry.update();

            if (gamepad1.b) isRedAlliance = true;
            else if (gamepad1.x) isRedAlliance = false;
        }

        shooter.initServos();
        drive.setAlliance(isRedAlliance);

        // Get goal position
        double goalX = isRedAlliance ? RobotConstants.GOAL_RED_X : RobotConstants.GOAL_BLUE_X;
        double goalY = isRedAlliance ? RobotConstants.GOAL_RED_Y : RobotConstants.GOAL_BLUE_Y;

        waitForStart();

        // Set turret to RUN_TO_POSITION mode
        shooter.setTurretRunToPositionMode();

        while (opModeIsActive()) {
            // ==================== CACHE HARDWARE READS ====================
            drive.cacheDriveVelocities();

            // ==================== SENSORS ====================
            double turretCurrentTicks = shooter.getTurretEncoderPos();

            // Calculate turret target from odometry
            double turretTargetAngle = drive.calculateTurretAngleToGoal(goalX, goalY);
            int turretTargetTicks = (int) shooter.angleToTurretTicks(turretTargetAngle);
            double cachedTurretError = Math.abs(turretTargetTicks - turretCurrentTicks);

            // Synchronous limelight update (only when turret is close to target)
            boolean hasCorrectTarget = false;
            double tx = 0.0;
            if (cachedTurretError < 100) {
                shooter.updateLimelightData(isRedAlliance);
                boolean hasTarget = shooter.hasLimelightTarget();
                int detectedTagId = shooter.getDetectedAprilTagId(isRedAlliance);
                tx = shooter.getLimelightTx(isRedAlliance);
                int expectedTagId = isRedAlliance ? 24 : 20;
                hasCorrectTarget = hasTarget && (detectedTagId == expectedTagId);
            } else {
                shooter.invalidateLimelightCache();
            }

            // Update odometry (unless visual tracking is active)
            if (!hasCorrectTarget) {
                drive.updateOdometry();
                turretTargetAngle = drive.calculateTurretAngleToGoal(goalX, goalY);
                turretTargetTicks = (int) shooter.angleToTurretTicks(turretTargetAngle);
                cachedTurretError = Math.abs(turretTargetTicks - turretCurrentTicks);
            }

            // ==================== DRIVETRAIN ====================
            double driveInput = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;
            drive.mecanumDriveWithBraking(driveInput, strafe, rotate, 1.0);

            // ==================== TURRET (RUN_TO_POSITION) ====================
            boolean rightStickButton = gamepad1.right_stick_button;
            if (rightStickButton && !lastRightStickButton) {
                shooter.setTurretRunWithoutEncoderMode();
                shooter.resetTurretZeroState();
                shooter.startTurretZero();
            }
            lastRightStickButton = rightStickButton;

            // Turret control using RUN_TO_POSITION
            boolean usingVisualTracking = false;
            if (shooter.isTurretZeroComplete()) {
                // Re-enable RUN_TO_POSITION after zeroing
                shooter.setTurretRunToPositionMode();
                
                if (hasCorrectTarget) {
                    // Visual tracking: adjust target by TX offset
                    shooter.setTurretTargetWithVisualOffset(turretTargetTicks, tx, TURRET_POWER);
                    usingVisualTracking = true;
                } else {
                    // Position-based: use odometry target
                    shooter.setTurretTargetPosition(turretTargetTicks, TURRET_POWER);
                }
            } else {
                shooter.updateTurretZero();
            }

            boolean turretOnTarget = shooter.isTurretAtTarget(TURRET_FIRE_TOLERANCE);

            // ==================== SHOOTING SEQUENCE ====================
            boolean leftBumper = gamepad1.left_bumper;
            boolean rightBumper = gamepad1.right_bumper;
            boolean shootButtonPressed = leftBumper || rightBumper;

            // Track spin-up time
            if (shootButtonPressed && !wasShootButtonPressed) {
                shooterSpinupStartTime = System.currentTimeMillis();
            }
            wasShootButtonPressed = shootButtonPressed;

            long spinupTime = shootButtonPressed ? (System.currentTimeMillis() - shooterSpinupStartTime) : 0;
            boolean hasMinSpinupTime = spinupTime >= MIN_SPINUP_TIME_MS;

            // Apply TPS override for bumpers (only used when no tag visible)
            if (leftBumper) {
                shooter.setDefaultTPSOverride(1800.0);
            } else if (rightBumper) {
                shooter.setDefaultTPSOverride(1600.0);
            } else {
                shooter.clearDefaultTPSOverride();
            }

            // Variable TPS based on distance (ty lookup)
            double currentTPS = shooter.getShooterTPS();
            double targetTPS = shooter.getTargetShooterTPS();
            boolean shooterReady = Math.abs(currentTPS - targetTPS) <= RobotConstants.SHOOTER_READY_THRESHOLD;

            double driveEncoderVelocity = drive.getCachedDriveVelocity();
            boolean isStationary = driveEncoderVelocity < STATIONARY_VELOCITY_THRESHOLD;

            // Latch logic
            if (!shootButtonPressed) {
                shootingLatched = false;
            } else if (!shootingLatched && hasMinSpinupTime && isStationary && shooterReady && turretOnTarget) {
                shootingLatched = true;
            }

            // Run shooter at variable TPS
            boolean feeding = false;
            if (shootButtonPressed || gamepad1.a) {
                // Bang-bang control at variable TPS
                double power = (currentTPS < targetTPS) ? RobotConstants.SHOOTER_MAX_POWER : 0.0;
                shooter.setShooterPower(power);
                SHOOTING = true;

                if (shootButtonPressed && shootingLatched) {
                    shooter.setBlocker(false);
                    if (!wasFeeding) {
                        feedingStartTime = System.currentTimeMillis();
                    }
                    shooter.setIntakePower(-1.0);
                    feeding = true;
                }
            } else {
                shooter.setShooterPower(0);
                SHOOTING = false;
            }
            wasFeeding = feeding;

            // ==================== INTAKE ====================
            if (gamepad1.left_trigger > 0.1) {
                shooter.runIntakeSystem(1.0);
                shooter.setBlocker(true);
                feeding = true;
            }

            if (gamepad1.right_trigger > 0.1) {
                shooter.runIntakeSystem(-1.0);
                shooter.setBlocker(true);
                feeding = true;
            }

            if (!feeding) {
                shooter.setIntakePower(-0.1);
                if (!shootButtonPressed) {
                    shooter.setBlocker(true);
                }
            }

            // ==================== LED ====================
            boolean hasThreeBalls = shooter.hasThreeBalls();
            shooter.updateLightServo(
                SHOOTING, shooterReady, isStationary,
                shootingLatched, hasCorrectTarget, turretOnTarget, usingVisualTracking, hasThreeBalls
            );

            // ==================== CLIMBER ====================
            boolean backButton = gamepad1.back;
            if (backButton && !lastBackButton) {
                climberUp = !climberUp;
                if (climberUp) shooter.setClimberUp();
                else shooter.setClimberDown();
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
            telemetry.addData("=== TEST MODE ===", "RUN_TO_POSITION | Variable TPS");
            
            telemetry.addData("Status", "%s | %s | %s",
                RobotConstants.getCurrentRobot(),
                isRedAlliance ? "RED" : "BLUE",
                SHOOTING ? "SHOOTING" : "IDLE");

            telemetry.addData("Shooter", "%.0f/%.0f TPS %s %s",
                currentTPS, targetTPS,
                shooterReady ? "[RDY]" : "",
                isStationary ? "[STOP]" : "[MOVING]");

            String turretMode = shooter.isTurretZeroComplete() ?
                (usingVisualTracking ? "[VIS-RTP]" : "[POS-RTP]") : "[ZEROING]";
            telemetry.addData("Turret", "%.0f/%.0f err:%.0f %s %s",
                turretCurrentTicks, (double)turretTargetTicks, cachedTurretError,
                turretOnTarget ? "[ON]" : "",
                turretMode);

            telemetry.addData("Limelight", "%s tx:%.1f",
                hasCorrectTarget ? "[OK]" : "---", tx);

            telemetry.addData("Odom", "X:%.1f Y:%.1f H:%.1f",
                drive.getOdometryX() / 25.4,
                drive.getOdometryY() / 25.4,
                drive.getOdometryHeading());

            telemetry.update();
        }

        // Cleanup
        shooter.setTurretRunWithoutEncoderMode();
        drive.stopMotors();
        shooter.stopAll();
    }
}

