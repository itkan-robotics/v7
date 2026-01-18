package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MainTeleop", group="Linear Opmode")
public class MainTeleOp extends LinearOpMode {

    // Robot functions (handles all hardware)
    private RobotFunctions robot;

    // Robot selection (21171 or 19564)
    private int selectedRobot = RobotConstants.ROBOT_19564;

    // Alliance tracking
    private boolean isRedAlliance = true;

    // Shooting state
    private boolean SHOOTING = false;
    private boolean shootingLatched = false;  // Once RPM is reached, stay shooting until conditions fail
    
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
    
    // Feeding timing for transfer power ramp
    private long feedingStartTime = 0;
    private boolean wasFeeding = false;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(5);
        
        // ==================== INITIALIZE HARDWARE FIRST ====================
        // Use default robot (21171) for initial hardware setup
        RobotConstants.setRobot(RobotConstants.ROBOT_21171);
        
        // Initialize all robot hardware and functions (servos NOT positioned yet)
        robot = new RobotFunctions(hardwareMap);
        
        // Reset pinpoint/odometry immediately so it calibrates during selection
        // No servo motion during this time for accurate IMU calibration
        robot.drivetrain.resetOdometry();
        
        // ==================== ROBOT & ALLIANCE SELECT ====================
        // Select robot and alliance while pinpoint calibrates
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("=== ROBOT SELECT ===", "");
            telemetry.addLine("Y = 21171  |  A = 19564");
            telemetry.addData("Robot", selectedRobot);
            telemetry.addLine("");
            telemetry.addData("=== ALLIANCE SELECT ===", "");
            telemetry.addLine("B = RED  |  X = BLUE");
            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("Target Tag", isRedAlliance ? 24 : 20);
            telemetry.addLine("");
            telemetry.addData("Pinpoint Status", robot.drivetrain.getPinpointStatus());
            telemetry.addData("Status", "Press START when ready");
            telemetry.update();

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
            robot.drivetrain.applyPinpointSettings();
            robot.shooter.applyMotorSettings();
        }
        
        // NOW initialize servos (after pinpoint IMU has calibrated)
        robot.initAllServos();
        
        // Set alliance for all functions
        robot.setAlliance(isRedAlliance);

        runtime.reset();

        waitForStart();

        while (opModeIsActive()) {
            // Update odometry
            robot.drivetrain.updateOdometry();

            // ==================== UPDATE LIMELIGHT DATA (once per loop) ====================
            robot.turret.updateLimelightData(isRedAlliance);
            
            // Get cached limelight values (no additional hardware reads)
            double tx = robot.turret.getLimelightTx(isRedAlliance);
            double ty = robot.turret.getLimelightTy();
            double ta = robot.turret.getAprilTagArea();
            int detectedTagId = robot.turret.getDetectedAprilTagId(isRedAlliance);
            boolean hasTarget = robot.turret.hasLimelightTarget();

            // ==================== MECANUM DRIVETRAIN ====================
            double drive = -gamepad1.left_stick_y;   // Forward/backward
            double strafe = gamepad1.left_stick_x;    // Left/right
                double rotate = -gamepad1.right_stick_x;  // Rotation

            // Calculate drivetrain input power for visual tracking check
            double driveInputPower = Math.max(Math.abs(drive), Math.max(Math.abs(strafe), Math.abs(rotate)));

            robot.drivetrain.mecanumDrive(drive, strafe, rotate, 1.0);


            // ==================== TURRET CONTROL ====================
            // Only allow visual tracking when drivetrain input < 50% and velocity < 1500mm/s
            boolean inputLowEnough = driveInputPower < 0.50;
            boolean velocityLowEnough = robot.drivetrain.getVelocityMagnitude() < 500;
            boolean allowVisualTracking = inputLowEnough && velocityLowEnough;
            
            // Point turret at goal
            robot.turret.pointTurretAtGoal(isRedAlliance, allowVisualTracking);

            // ==================== SHOOTER CONTROL ====================
            // Dpad up/down: Toggle manual RPM mode and adjust TPS (edge-triggered)
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
            
            // A button: spin up shooter to target RPM (just motor, no feeding)
            if (gamepad1.a) {
                if (manualRPMMode) {
                    robot.shooter.controlShooterManual(manualTargetTPS, true);
                } else {
                    robot.shooter.controlShooter(true);
                }
            }
            
            // ==================== BUMPER SHOOTING (PRESET POSITIONS) ====================
            // Left bumper: Preset position (72, 24) with default TPS 1500 (close shot)
            // Right bumper: Preset position (72, 72) with default TPS 1750 (far shot)
            // Note: Default TPS is only used when no AprilTag visible; ty-based TPS used when tracking
            boolean leftBumper = gamepad1.left_bumper;
            boolean rightBumper = gamepad1.right_bumper;
            boolean shootButtonPressed = leftBumper || rightBumper;
            
            // Set position override and default TPS override based on which bumper is pressed
            if (leftBumper) {
                robot.drivetrain.setPositionOverride(72.0, 30.0);
                robot.shooter.setDefaultTPSOverride(1750.0);
                robot.turret.setCloseShotOverride(false);
            } else if (rightBumper) {
                robot.drivetrain.setPositionOverride(72.0, 90.0);
                robot.shooter.setDefaultTPSOverride(1550.0);
                robot.turret.setCloseShotOverride(true);  // Force target TX to 0 for close shot
            } else {
                robot.drivetrain.clearPositionOverride();
                robot.shooter.clearDefaultTPSOverride();
                robot.turret.setCloseShotOverride(false);
            }

            double currentTPS = robot.shooter.getShooterTPS();
            // Target TPS: use manual if in manual mode, otherwise auto (ty-based or default override)
            double targetTPS;
            if (manualRPMMode) {
                targetTPS = manualTargetTPS;
            } else {
                targetTPS = robot.shooter.getTargetShooterTPS();
            }
            
            // Apply far shot tx offset when TPS > 1650 (only for 21171)
            if (RobotConstants.hasFarShotTxOffset() && targetTPS > 1650) {
                robot.turret.setFarShotTxOffset(2.5);
            } else {
                robot.turret.clearFarShotTxOffset();
            }
            
            boolean shooterReady = robot.shooter.isShooterSpeedReady(targetTPS);
            boolean turretOnTarget = robot.turret.isTurretOnTarget(isRedAlliance);

            // ==================== INTAKE/TRANSFER CONTROLS ====================
            boolean feeding = false;

            // Bumper shooting logic with latch:
            // - Wait for RPM to reach target initially
            // - Once reached, keep shooting until bumper released OR alignment wrong
            if (!shootButtonPressed) {
                // Reset latch when button released
                shootingLatched = false;
            } else {
                // Button is held - check if we should start/continue shooting
                if (shootingLatched) {
                    // Already shooting - check if we should stop
                    if (!turretOnTarget) {
                        // Alignment lost - stop shooting
                        shootingLatched = false;
                    }
                } else {
                    // Not yet shooting - latch immediately when ready
                    if (shooterReady) {
                        shootingLatched = true;
                    }
                }
            }
            
            // Instant unblock when bumper pressed
            boolean turretVisual = robot.turret.isTurretUsingVisualTracking();
            if (shootButtonPressed) {
                robot.intake.setBlocker(false);  // Unblock instantly
            }
            
            // Feed if latched and aligned (intake/transfer still wait for conditions)
            if (shootButtonPressed && shootingLatched && turretOnTarget) {
                // Track when feeding started
                if (!wasFeeding) {
                    feedingStartTime = System.currentTimeMillis();
                }
                
                // Feed balls toward shooter
                robot.intake.setIntakePower(-1.0);
                
                // For far shots (high RPM): use 100% power for first 0.25sec, then 65%
                // For close shots: always use 100%
                double transferPower;
                if (targetTPS > 1650) {
                    long feedingDuration = System.currentTimeMillis() - feedingStartTime;
                    if (feedingDuration < 250) {
                        transferPower = -1.0;  // Full power for first 0.25 sec
                    } else {
                        transferPower = -0.65;  // Reduced power after
                    }
                } else {
                    transferPower = -1.0;  // Full power for close shots
                }
                robot.intake.setTransferPower(transferPower);
                feeding = true;
            }
            wasFeeding = feeding;
            
            // Shooter control - must be after feeding is determined
            if (shootButtonPressed) {
                // Spin up shooter (uses auto TPS with default override when no tag visible)
                if (manualRPMMode) {
                    robot.shooter.controlShooterManual(manualTargetTPS, true, feeding);
                } else {
                    robot.shooter.controlShooter(true, feeding);
                }
                SHOOTING = true;
            } else if (!gamepad1.a) {
                // Only stop shooter if neither A nor bumpers are pressed
                robot.shooter.controlShooter(false);
                SHOOTING = false;
            }

            // Main controller reverse/unjam (left trigger)
            if (gamepad1.left_trigger > 0.1) {
                robot.intake.runIntakeSystem(1.0);
                robot.intake.setBlocker(true);
                feeding = true;
            }

            // Main controller intake (right trigger)
            if (gamepad1.right_trigger > 0.1) {
                robot.intake.runIntakeSystem(-1.0);
                robot.intake.setBlocker(true);
                feeding = true;
            }


            // Default intake/transfer state when not feeding
            if (!feeding) {
                robot.intake.setIntakePower(-0.1);  // Light inward hold
                robot.intake.setTransferPower(0.0);
                // Only block if left bumper is not pressed (left bumper keeps it unblocked)
                if (!shootButtonPressed) {
                    robot.intake.setBlocker(true);
                }
            }

            // ==================== LED FEEDBACK ====================
            boolean hasThreeBalls = robot.intake.hasThreeBalls();
            
            robot.led.updateLightServo(
                SHOOTING,           // shooterRunning
                shooterReady,       // shooterReady
                feeding,            // intakeRunning
                feeding && SHOOTING && shooterReady,  // isShooting
                detectedTagId > 0,  // aprilTagVisible
                turretOnTarget,     // turretOnTarget
                turretVisual,       // turretUsingVisualTracking
                hasThreeBalls       // hasThreeBalls (transfer current > 5A)
            );


            // ==================== CLIMBER CONTROL ====================
            // Gamepad1 back button - toggle climber position
            boolean backButton = gamepad1.back;
            if (backButton && !lastBackButton) {
                // Rising edge - toggle climber
                climberUp = !climberUp;
                if (climberUp) {
                    robot.climber.setClimberUp();
                } else {
                    robot.climber.setClimberDown();
                }
            }
            lastBackButton = backButton;

            // ==================== ODOMETRY RESET ====================
            // Left stick button - reset position to field center with heading 270
            boolean leftStickButton = gamepad1.left_stick_button;
            if (leftStickButton && !lastLeftStickButton) {
                // Rising edge - reset odometry to field center with 270 heading
                double fieldCenterX = DrivetrainFunctions.FIELD_CENTER_X_INCHES * DrivetrainFunctions.INCHES_TO_MM;
                double fieldCenterY = DrivetrainFunctions.FIELD_CENTER_Y_INCHES * DrivetrainFunctions.INCHES_TO_MM;
                robot.drivetrain.setOdometryPosition(fieldCenterX, fieldCenterY, 270.0);
            }
            lastLeftStickButton = leftStickButton;

            // ==================== INDEXER CONTROL ====================
            // Operator dpad_left - indexed position
            if (gamepad2.dpad_left) {
                robot.indexer.setIndexerIndexed();
            }
            // Operator dpad_right - middle position
            if (gamepad2.dpad_right) {
                robot.indexer.setIndexerMiddle();
            }

            // ==================== TELEMETRY ====================
            telemetry.addData("=== STATUS ===", "");
            telemetry.addData("Robot", RobotConstants.getCurrentRobot());
            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("Shooting", SHOOTING);

            telemetry.addData("=== SHOOTER ===", "");
            telemetry.addData("Mode", manualRPMMode ? "MANUAL (Dpad ↑↓)" : "AUTO (Limelight)");
            telemetry.addData("Velocity", "%.0f / %.0f TPS", currentTPS, targetTPS);
            telemetry.addData("Latched", shootingLatched ? "YES" : "NO");
            telemetry.addData("Power", "%.1f", robot.shooter.getShooterPower());
            telemetry.addData("Ready", shooterReady ? "YES" : "NO");

            telemetry.addData("=== TURRET DEBUG ===", "");
            telemetry.addData("Mode", turretVisual ? "VISUAL (Limelight)" : "POSITION (Odometry)");
            if (!turretVisual) {
                // Show why visual tracking is disabled
                if (!inputLowEnough) {
                    telemetry.addData("Visual OFF", "Input too high (%.0f%%)", driveInputPower * 100);
                } else if (!velocityLowEnough) {
                    telemetry.addData("Visual OFF", "Velocity too high (%.0f mm/s)", robot.drivetrain.getVelocityMagnitude());
                } else if (detectedTagId <= 0) {
                    telemetry.addData("Visual OFF", "No AprilTag detected");
                } else {
                    telemetry.addData("Visual OFF", "Wrong tag (got %d)", detectedTagId);
                }
            }
            telemetry.addData("Tx Offset", "%.2f°", robot.turret.getTargetTxOffset());
            telemetry.addData("Robot X/Y", "%.1f / %.1f in", 
                robot.drivetrain.getOdometryX() / 25.4, 
                robot.drivetrain.getOdometryY() / 25.4);
            telemetry.addData("Predicted X/Y", "%.1f / %.1f in", 
                robot.drivetrain.getPredictedX() / 25.4, 
                robot.drivetrain.getPredictedY() / 25.4);
            telemetry.addData("Velocity", "%.0f mm/s", robot.drivetrain.getVelocityMagnitude());
            telemetry.addData("Robot Heading", "%.1f°", robot.drivetrain.getOdometryHeading());
            telemetry.addData("Turret Angle", "%.1f°", robot.turret.getTurretAngle());
            telemetry.addData("On Target", turretOnTarget ? "YES" : "NO");

            telemetry.addData("=== LIMELIGHT ===", "");
            if (hasTarget) {
                telemetry.addData("Tag ID", detectedTagId);
                telemetry.addData("tx", "%.2f°", tx);
                telemetry.addData("ty", "%.2f°", ty);
                telemetry.addData("Area (ta)", "%.3f%%", ta);
                telemetry.addData("Distance", "%.1f in", 
                    robot.turret.getAprilTagDistance(isRedAlliance) / 25.4);
            } else {
                telemetry.addData("Target", "NONE");
            }

            telemetry.addData("=== ODOMETRY ===", "");
            telemetry.addData("Position", "X:%.1f Y:%.1f in", 
                robot.drivetrain.getOdometryX() / 25.4,
                robot.drivetrain.getOdometryY() / 25.4);
            telemetry.addData("Heading", "%.1f°", robot.drivetrain.getOdometryHeading());

            telemetry.update();
        }

        // Stop everything
        robot.stopAll();
    }
}
