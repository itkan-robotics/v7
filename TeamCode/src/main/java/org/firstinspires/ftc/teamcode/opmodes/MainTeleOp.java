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

    // ==================== TURRET PID TUNING (Panels) ====================
    // Position-based turret control
    public static double turretKp = RobotConstants.TURRET_KP;  // 0.05
    
    // Visual tracking PID
    public static double turretVisualKp = RobotConstants.TURRET_VISUAL_KP;  // 0.02
    public static double turretVisualKd = RobotConstants.TURRET_VISUAL_KD;  // 0.05
    public static double turretVisualKf = RobotConstants.TURRET_VISUAL_KF;  // 0.0
    public static double turretVisualDeadband = RobotConstants.TURRET_VISUAL_DEADBAND;  // 0.65
    public static double turretVisualMaxPower = RobotConstants.TURRET_VISUAL_MAX_POWER;  // 0.6
    
    // Turn feedforward
    public static double turretTurnFf = RobotConstants.TURRET_TURN_FF;  // -0.5
    public static double turretTurnFfDecay = RobotConstants.TURRET_TURN_FF_DECAY;  // 3.0
    
    // Limelight engage threshold
    public static double turretLimelightThreshold = RobotConstants.TURRET_LIMELIGHT_THRESHOLD;  // 50.0

    // ==================== TURRET FSM ====================
    private enum TurretState {
        ZEROING,           // Turret is finding home position
        POSITION_TRACKING, // Using odometry to point at goal
        VISUAL_TRACKING    // Using limelight to fine-tune aim
    }
    private TurretState turretState = TurretState.ZEROING;
    
    // Threshold to switch from position to visual tracking
    private static final double VISUAL_ENGAGE_THRESHOLD = 75.0;

    // Panels telemetry manager
    private TelemetryManager panelsTelemetry;

    // Subsystems
    private Drive drive;
    private Shooter shooter;

    // Alliance tracking
    private boolean isRedAlliance = true;

    // Shooting state
    private boolean SHOOTING = false;
    private boolean shootingLatched = false;

    // Climber toggle
    private boolean climberUp = false;
    private boolean lastBackButton = false;

    // Odometry reset
    private boolean lastLeftStickButton = false;
    
    // Turret zero
    private boolean lastRightStickButton = false;

    // Feeding timing for transfer power ramp
    private long feedingStartTime = 0;
    private boolean wasFeeding = false;
    
    // Loop timing for turn FF decay
    private long lastLoopTime = 0;
    
    // Stationary check for shooting (uses drive encoder velocity, not odometry)
    private static final double STATIONARY_VELOCITY_THRESHOLD = 100.0;
    
    // Turret tolerance for fire sequence
    private static final double TURRET_FIRE_TOLERANCE = 50.0;
    
    // Minimum spin-up time before allowing fire (prevents firing while coasting)
    private static final long MIN_SPINUP_TIME_MS = 200;
    private long shooterSpinupStartTime = 0;
    private boolean wasShootButtonPressed = false;
    
    // Minimum time all conditions must be ready before firing
    private static final long MIN_CONDITIONS_READY_TIME_MS = 30;
    private long conditionsReadyStartTime = 0;
    private boolean conditionsWereReady = false;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // ==================== INITIALIZE HARDWARE ====================
        RobotConstants.setRobot(RobotConstants.ROBOT_19564);

        // Initialize subsystems
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);

        // Initialize odometry
        drive.updateOdometry();

        // Start turret zeroing during init
        shooter.startTurretZero();
        turretState = TurretState.ZEROING;

        // ==================== ALLIANCE SELECT ====================
        while (!opModeIsActive() && !isStopRequested()) {
            shooter.updateTurretZero();
            
            telemetry.addData("=== 19564 SOLAR ROBOT ===", "");
            telemetry.addLine("");
            telemetry.addData("=== ALLIANCE SELECT ===", "");
            telemetry.addLine("B = RED  |  X = BLUE");
            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("Target Tag", isRedAlliance ? 24 : 20);
            telemetry.addLine("");
            telemetry.addData("Pinpoint Status", drive.getPinpointStatus());
            telemetry.addData("Turret Zero", shooter.isTurretZeroComplete() ? "COMPLETE" : "IN PROGRESS...");
            telemetry.addLine("");
            telemetry.addData("=== TURRET PID (Panels) ===", "");
            telemetry.addData("Pos Kp", "%.4f", turretKp);
            telemetry.addData("Vis Kp/Kd/Kf", "%.4f / %.4f / %.4f", turretVisualKp, turretVisualKd, turretVisualKf);
            telemetry.addData("Vis Deadband/MaxPwr", "%.2f / %.2f", turretVisualDeadband, turretVisualMaxPower);
            telemetry.addData("Turn FF/Decay", "%.2f / %.2f", turretTurnFf, turretTurnFfDecay);
            telemetry.addData("LL Threshold", "%.1f", turretLimelightThreshold);
            telemetry.addLine("");
            telemetry.addData("Status", "Press START when ready");
            telemetry.update();
            panelsTelemetry.update(telemetry);

            if (gamepad1.b) isRedAlliance = true;
            else if (gamepad1.x) isRedAlliance = false;
        }

        // Initialize servos and set alliance
        shooter.initServos();
        drive.setAlliance(isRedAlliance);

        // Get goal position for turret tracking
        double goalX = isRedAlliance ? RobotConstants.GOAL_RED_X : RobotConstants.GOAL_BLUE_X;
        double goalY = isRedAlliance ? RobotConstants.GOAL_RED_Y : RobotConstants.GOAL_BLUE_Y;
        int expectedTagId = isRedAlliance ? 24 : 20;

        waitForStart();
        lastLoopTime = System.nanoTime();
        
        // Start background limelight thread
        shooter.startLimelightThread(isRedAlliance);

        while (opModeIsActive()) {
            // ==================== LOOP TIMING ====================
            long loopStart = System.nanoTime();
            double deltaTimeSec = (loopStart - lastLoopTime) / 1_000_000_000.0;
            if (deltaTimeSec > 0.1) deltaTimeSec = 0.1;  // Cap for first frame
            lastLoopTime = loopStart;
            
            // ==================== CACHE HARDWARE READS ====================
            drive.cacheDriveVelocities();
            double turretCurrentTicks = shooter.getTurretEncoderPos();
            
            // ==================== APPLY TURRET PID OVERRIDES (from Panels) ====================
            shooter.setAllTurretOverrides(
                turretKp, turretVisualKp, turretVisualKd, turretVisualKf,
                turretTurnFf, turretTurnFfDecay, turretVisualDeadband,
                turretVisualMaxPower, turretLimelightThreshold
            );
            
            // ==================== TURRET FSM ====================
            // Calculate position-based target (always needed for error calculation)
            double turretTargetAngle = drive.calculateTurretAngleToGoal(goalX, goalY);
            double turretTargetTicks = shooter.angleToTurretTicks(turretTargetAngle);
            double turretError = Math.abs(turretTargetTicks - turretCurrentTicks);
            
            // Limelight and visual tracking variables
            boolean hasCorrectTarget = false;
            double tx = 0.0;
            double ty = 0.0;
            double turretPower = 0.0;
            
            // Update turn feedforward
            double rotate = -gamepad1.right_stick_x;
            shooter.updateTurnFF(rotate, deltaTimeSec);
            
            // Manual turret zero request
            boolean rightStickButton = gamepad1.right_stick_button;
            if (rightStickButton && !lastRightStickButton) {
                shooter.resetTurretZeroState();
                shooter.startTurretZero();
                turretState = TurretState.ZEROING;
            }
            lastRightStickButton = rightStickButton;
            
            // FSM switch statement
            switch (turretState) {
                case ZEROING:
                    // Keep updating zero routine
                    shooter.updateTurretZero();
                    
                    // Transition: zeroing complete → position tracking
                    if (shooter.isTurretZeroComplete()) {
                        turretState = TurretState.POSITION_TRACKING;
                    }
                    
                    // Always update odometry while zeroing
                    drive.updateOdometry();
                    break;
                    
                case POSITION_TRACKING:
                    // Always update odometry in this state
                    drive.updateOdometry();
                    
                    // Recalculate target with fresh odometry
                    turretTargetAngle = drive.calculateTurretAngleToGoal(goalX, goalY);
                    turretTargetTicks = shooter.angleToTurretTicks(turretTargetAngle);


                    // Control turret with position-based tracking
                    Shooter.TurretControlResult posResult = shooter.updateTurretControl(false, 0, turretTargetTicks);
                    turretPower = posResult.power;
                    
                    // Transition check: when close enough, check limelight (from background thread)
                    if (turretError < VISUAL_ENGAGE_THRESHOLD) {
                        // Read cached limelight data (updated by background thread)
                        boolean hasTarget = shooter.hasLimelightTarget();
                        int detectedTagId = shooter.getDetectedAprilTagId(isRedAlliance);
                        
                        if (hasTarget && detectedTagId == expectedTagId) {
                            turretState = TurretState.VISUAL_TRACKING;
                        }
                    }
                    break;
                    
                case VISUAL_TRACKING:
                    // DO NOT update odometry - freeze it while visual tracking
                    drive.updateOdometry();

                    // Read cached limelight data (updated by background thread)
                    boolean hasTarget = shooter.hasLimelightTarget();
                    int detectedTagId = shooter.getDetectedAprilTagId(isRedAlliance);
                    tx = shooter.getLimelightTx(isRedAlliance);
                    ty = shooter.getLimelightTy();
                    
                    hasCorrectTarget = hasTarget && (detectedTagId == expectedTagId);
                    
                    // Transition check: lost target → back to position tracking
                    if (!hasCorrectTarget) {
                        turretState = TurretState.POSITION_TRACKING;
                        // Update odometry now that we're leaving visual tracking
                        drive.updateOdometry();
                        break;
                    }
                    
                    // Control turret with visual tracking (aim for tx = 0)
                    double visualError = 0.0 - tx;
                    Shooter.TurretControlResult visResult = shooter.updateTurretControl(true, visualError, turretTargetTicks);
                    turretPower = visResult.power;
                    break;
            }

            // ==================== DRIVETRAIN ====================
            double driveInput = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            drive.mecanumDrive(driveInput, strafe, rotate, 1.0);

            // ==================== SHOOTING SEQUENCE ====================
            boolean leftBumper = gamepad1.left_bumper;
            boolean rightBumper = gamepad1.right_bumper;
            boolean shootButtonPressed = leftBumper || rightBumper;

            // Track when shoot button was first pressed (for spin-up timing)
            if (shootButtonPressed && !wasShootButtonPressed) {
                shooterSpinupStartTime = System.currentTimeMillis();
            }
            wasShootButtonPressed = shootButtonPressed;
            
            // Check if shooter has had minimum spin-up time
            long spinupTime = shootButtonPressed ? (System.currentTimeMillis() - shooterSpinupStartTime) : 0;
            boolean hasMinSpinupTime = spinupTime >= MIN_SPINUP_TIME_MS;

            // Apply TPS override (only used when no tag visible)
            if (leftBumper) {
                shooter.setDefaultTPSOverride(1750.0);
                shooter.setTargetTxOffset(-2);
            } else if (rightBumper) {
                shooter.setDefaultTPSOverride(1550.0);
                shooter.setTargetTxOffset(0);
            } else {
                shooter.clearDefaultTPSOverride();
            }

            // Get TPS values once for use throughout this loop iteration
            double currentTPS = shooter.getShooterTPS();
            double targetTPS = shooter.getTargetShooterTPS();
            boolean shooterReady = shooter.isShooterSpeedReady(targetTPS);

            
            double driveEncoderVelocity = drive.getCachedDriveVelocity();
            boolean isStationary = driveEncoderVelocity < STATIONARY_VELOCITY_THRESHOLD;
            boolean turretOnTarget = shooter.isTurretAligned(isRedAlliance);
            // Check all firing conditions
            boolean allConditionsReady = hasMinSpinupTime && isStationary && shooterReady && turretOnTarget;
            
            // Track how long conditions have been continuously ready
            if (allConditionsReady && !conditionsWereReady) {
                conditionsReadyStartTime = System.currentTimeMillis();
            }
            if (!allConditionsReady) {
                conditionsReadyStartTime = 0;
            }
            conditionsWereReady = allConditionsReady;
            
            long conditionsReadyDuration = allConditionsReady ? (System.currentTimeMillis() - conditionsReadyStartTime) : 0;
            boolean conditionsReadyLongEnough = conditionsReadyDuration >= MIN_CONDITIONS_READY_TIME_MS;

            // Reset latch when button released
            if (!shootButtonPressed) {
                // If we were firing (latched), zero turret to counter encoder drift
                if (shootingLatched) {
                    shooter.resetTurretZeroState();
                    shooter.startTurretZero(0.65);  // Faster zeroing during run
                    turretState = TurretState.ZEROING;
                }
                shootingLatched = false;
            }
            // Latch when ALL conditions met for required duration (30ms)
            else if (!shootingLatched && conditionsReadyLongEnough) {
                shootingLatched = true;
            }

            // Run shooter and feeding
            boolean feeding = false;
            if (shootButtonPressed) {
                // Run shooter motor with pre-computed TPS values
                shooter.controlShooter(true, currentTPS, targetTPS);
                SHOOTING = true;

                // Once latched, continue feeding regardless of other conditions
                if (shootingLatched) {
                    shooter.setBlocker(false);
                    if (!wasFeeding) {
                        feedingStartTime = System.currentTimeMillis();
                    }
                    
                    double transferPower = -1.0;
                    if (targetTPS > 1650) {
                        long feedingDuration = System.currentTimeMillis() - feedingStartTime;
                        if (feedingDuration >= 250) {
                            transferPower = -0.65;
                        }
                    }
                    shooter.setIntakePower(transferPower);
                    feeding = true;
                }
            } else if (gamepad1.a) {
                // A button spins up shooter without feeding
                shooter.controlShooter(true, currentTPS, targetTPS);
                SHOOTING = true;
            } else {
                shooter.controlShooter(false, currentTPS, targetTPS);
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
            // LED KEY: WHITE=firing, BLUE=ready, GREEN=turret aligned, YELLOW=visual tracking,
            //          ORANGE=spinning/not aligned, PURPLE=3 balls, RED=idle
            boolean hasThreeBalls = shooter.hasThreeBalls();
            boolean usingVisualTracking = (turretState == TurretState.VISUAL_TRACKING);
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
            telemetry.addData("Status", "%s | %s | %s", 
                RobotConstants.getCurrentRobot(),
                isRedAlliance ? "RED" : "BLUE",
                SHOOTING ? "SHOOTING" : "IDLE");
            
            telemetry.addData("Shooter", "%.0f/%.0f TPS %s %s", 
                currentTPS, targetTPS,
                shooterReady ? "[RDY]" : "",
                isStationary ? "[STOP]" : "[MOVING]");
            
            String turretStateStr = turretState.toString();
            double smoothedFF = shooter.getSmoothedTurnFF();
            telemetry.addData("Turret", "%.0f/%.0f err:%.0f pwr:%.3f ff:%.3f [%s]",
                turretCurrentTicks, turretTargetTicks, turretError,
                turretPower, smoothedFF, turretStateStr);
            
            telemetry.addData("Limelight", "%s tx:%.1f ty:%.1f",
                hasCorrectTarget ? "[OK]" : "---",
                tx, ty);
            
            telemetry.addData("Odom", "X:%.1f Y:%.1f H:%.1f %s",
                drive.getOdometryX() / 25.4, 
                drive.getOdometryY() / 25.4,
                drive.getOdometryHeading(),
                (turretState == TurretState.VISUAL_TRACKING) ? "[FROZEN]" : "");
            
            telemetry.addLine("");
            telemetry.addData("=== TURRET PID (Panels) ===", "");
            telemetry.addData("Pos Kp", "%.4f", turretKp);
            telemetry.addData("Vis Kp/Kd/Kf", "%.4f / %.4f / %.4f", turretVisualKp, turretVisualKd, turretVisualKf);
            telemetry.addData("Turn FF/Decay", "%.2f / %.2f", turretTurnFf, turretTurnFfDecay);

            telemetry.update();
            panelsTelemetry.update(telemetry);
        }
        
        drive.stopMotors();
        shooter.stopAll();
    }
    
}
