package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name="MainTeleop1", group="Linear Opmode")
public class MainTeleOp extends LinearOpMode {

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

    // Feeding timing for transfer power ramp
    private long feedingStartTime = 0;
    private boolean wasFeeding = false;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(5);

        // ==================== INITIALIZE HARDWARE FIRST ====================
        RobotConstants.setRobot(RobotConstants.ROBOT_21171);

        // Initialize subsystems
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap, drive);

        // Reset pinpoint/odometry immediately so it calibrates during selection
       // drive.resetOdometry();

        // ==================== ROBOT & ALLIANCE SELECT ====================
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
            telemetry.addData("Pinpoint Status", drive.getPinpointStatus());
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
            drive.applyPinpointSettings();
            shooter.applyMotorSettings();
        }

        // If auto ran, use the saved alliance so pinpoint math is correct
        // (Pinpoint keeps tracking from auto, so we need same starting offset)
        if (PoseStorage.hasSavedPose()) {
            isRedAlliance = PoseStorage.isRedAlliance();
            telemetry.addData("Position Source", "Continuous from Auto");
        } else {
            telemetry.addData("Position Source", "Default (no auto)");
        }
        telemetry.update();

        // NOW initialize servos (after pinpoint IMU has calibrated)
        shooter.initServos();

        // Set alliance
        drive.setAlliance(isRedAlliance);

        runtime.reset();
        waitForStart();

        while (opModeIsActive()) {
            // Update odometry
            drive.updateOdometry();

            // ==================== UPDATE LIMELIGHT DATA (once per loop) ====================
            shooter.updateLimelightData(isRedAlliance);

            // Get cached limelight values
            double tx = shooter.getLimelightTx(isRedAlliance);
            double ty = shooter.getLimelightTy();
            double ta = shooter.getAprilTagArea();
            int detectedTagId = shooter.getDetectedAprilTagId(isRedAlliance);
            boolean hasTarget = shooter.hasLimelightTarget();

            // ==================== MECANUM DRIVETRAIN ====================
            double driveInput = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;

            double driveInputPower = Math.max(Math.abs(driveInput), Math.max(Math.abs(strafe), Math.abs(rotate)));

            drive.mecanumDrive(driveInput, strafe, rotate, 1.0);

            // ==================== TURRET CONTROL ====================
            boolean inputLowEnough = driveInputPower < 0.50;
            boolean velocityLowEnough = drive.getVelocityMagnitude() < 500;
            boolean allowVisualTracking = inputLowEnough && velocityLowEnough;

            shooter.pointTurretAtGoal(isRedAlliance, allowVisualTracking);

            // ==================== SHOOTER CONTROL ====================
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

            // A button: spin up shooter to target RPM
            if (gamepad1.a) {
                if (manualRPMMode) {
                    shooter.controlShooterManual(manualTargetTPS, true);
                } else {
                    shooter.controlShooter(true);
                }
            }

            // ==================== BUMPER SHOOTING (PRESET POSITIONS) ====================
            boolean leftBumper = gamepad1.left_bumper;
            boolean rightBumper = gamepad1.right_bumper;
            boolean shootButtonPressed = leftBumper || rightBumper;

            if (leftBumper) {
                drive.setPositionOverride(72.0, 30.0);
                shooter.setDefaultTPSOverride(1750.0);
                shooter.setCloseShotOverride(false);
            } else if (rightBumper) {
                drive.setPositionOverride(72.0, 90.0);
                shooter.setDefaultTPSOverride(1550.0);
                shooter.setCloseShotOverride(true);
            } else {
                drive.clearPositionOverride();
                shooter.clearDefaultTPSOverride();
                shooter.setCloseShotOverride(false);
            }

            double currentTPS = shooter.getShooterTPS();
            double targetTPS;
            if (manualRPMMode) {
                targetTPS = manualTargetTPS;
            } else {
                targetTPS = shooter.getTargetShooterTPS();
            }

            // Apply far shot tx offset when TPS > 1650 (only for 21171)
            if (RobotConstants.hasFarShotTxOffset() && targetTPS > 1650) {
                shooter.setFarShotTxOffset(2.5);
            } else {
                shooter.clearFarShotTxOffset();
            }

            boolean shooterReady = shooter.isShooterSpeedReady(targetTPS);
            boolean turretOnTarget = shooter.isTurretOnTarget(isRedAlliance);

            // ==================== INTAKE/TRANSFER CONTROLS ====================
            boolean feeding = false;

            if (!shootButtonPressed) {
                shootingLatched = false;
            } else {
                if (shootingLatched) {
                    if (!turretOnTarget) {
                        shootingLatched = false;
                    }
                } else {
                    if (shooterReady) {
                        shootingLatched = true;
                    }
                }
            }

            boolean turretVisual = shooter.isTurretUsingVisualTracking();
            if (shootButtonPressed) {
                shooter.setBlocker(false);
            }

            if (shootButtonPressed && shootingLatched && turretOnTarget) {
                if (!wasFeeding) {
                    feedingStartTime = System.currentTimeMillis();
                }

                shooter.setIntakePower(-1.0);

                double transferPower;
                if (targetTPS > 1650) {
                    long feedingDuration = System.currentTimeMillis() - feedingStartTime;
                    if (feedingDuration < 250) {
                        transferPower = -1.0;
                    } else {
                        transferPower = -0.65;
                    }
                } else {
                    transferPower = -1.0;
                }
                shooter.setTransferPower(transferPower);
                feeding = true;
            }
            wasFeeding = feeding;

            // Shooter control
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

            // Main controller reverse/unjam (left trigger)
            if (gamepad1.left_trigger > 0.1) {
                shooter.runIntakeSystem(1.0);
                shooter.setBlocker(true);
                feeding = true;
            }

            // Main controller intake (right trigger)
            if (gamepad1.right_trigger > 0.1) {
                shooter.runIntakeSystem(-1.0);
                shooter.setBlocker(true);
                feeding = true;
            }

            // Default intake/transfer state when not feeding
            if (!feeding) {
                shooter.setIntakePower(-0.1);
                if (!shootButtonPressed) {
                    shooter.setBlocker(true);
                }
            }

            // ==================== LED FEEDBACK ====================
            boolean hasThreeBalls = shooter.hasThreeBalls();

            shooter.updateLightServo(
                SHOOTING,
                shooterReady,
                feeding,
                feeding && SHOOTING && shooterReady,
                detectedTagId > 0,
                turretOnTarget,
                turretVisual,
                hasThreeBalls
            );

            // ==================== CLIMBER CONTROL ====================
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
                double fieldCenterX = Drive.FIELD_CENTER_X_INCHES * Drive.INCHES_TO_MM;
                double fieldCenterY = Drive.FIELD_CENTER_Y_INCHES * Drive.INCHES_TO_MM;
                drive.setOdometryPosition(fieldCenterX, fieldCenterY, 270.0);
            }
            lastLeftStickButton = leftStickButton;
            

            // ==================== TELEMETRY ====================
            telemetry.addData("=== STATUS ===", "");
            telemetry.addData("Robot", RobotConstants.getCurrentRobot());
            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("Shooting", SHOOTING);

            telemetry.addData("=== SHOOTER ===", "");
            telemetry.addData("Mode", manualRPMMode ? "MANUAL (Dpad ↑↓)" : "AUTO (Limelight)");
            telemetry.addData("Velocity", "%.0f / %.0f TPS", currentTPS, targetTPS);
            telemetry.addData("Latched", shootingLatched ? "YES" : "NO");
            telemetry.addData("Power", "%.1f", shooter.getShooterPower());
            telemetry.addData("Ready", shooterReady ? "YES" : "NO");

            telemetry.addData("=== TURRET DEBUG ===", "");
            telemetry.addData("Mode", turretVisual ? "VISUAL (Limelight)" : "POSITION (Odometry)");
            if (!turretVisual) {
                if (!inputLowEnough) {
                    telemetry.addData("Visual OFF", "Input too high (%.0f%%)", driveInputPower * 100);
                } else if (!velocityLowEnough) {
                    telemetry.addData("Visual OFF", "Velocity too high (%.0f mm/s)", drive.getVelocityMagnitude());
                } else if (detectedTagId <= 0) {
                    telemetry.addData("Visual OFF", "No AprilTag detected");
                } else {
                    telemetry.addData("Visual OFF", "Wrong tag (got %d)", detectedTagId);
                }
            }
            telemetry.addData("Tx Offset", "%.2f°", shooter.getTargetTxOffset());
            telemetry.addData("Robot X/Y", "%.1f / %.1f in",
                drive.getOdometryX() / 25.4,
                drive.getOdometryY() / 25.4);
            telemetry.addData("Predicted X/Y", "%.1f / %.1f in",
                drive.getPredictedX() / 25.4,
                drive.getPredictedY() / 25.4);
            telemetry.addData("Velocity", "%.0f mm/s", drive.getVelocityMagnitude());
            telemetry.addData("Robot Heading", "%.1f°", drive.getOdometryHeading());
            telemetry.addData("Turret Angle", "%.1f°", shooter.getTurretAngle());
            telemetry.addData("On Target", turretOnTarget ? "YES" : "NO");

            telemetry.addData("=== LIMELIGHT ===", "");
            if (hasTarget) {
                telemetry.addData("Tag ID", detectedTagId);
                telemetry.addData("tx", "%.2f°", tx);
                telemetry.addData("ty", "%.2f°", ty);
                telemetry.addData("Area (ta)", "%.3f%%", ta);
                telemetry.addData("Distance", "%.1f in",
                    shooter.getAprilTagDistance(isRedAlliance) / 25.4);
            } else {
                telemetry.addData("Target", "NONE");
            }

            telemetry.addData("=== ODOMETRY ===", "");
            telemetry.addData("Position", "X:%.1f Y:%.1f in",
                drive.getOdometryX() / 25.4,
                drive.getOdometryY() / 25.4);
            telemetry.addData("Heading", "%.1f°", drive.getOdometryHeading());

            telemetry.update();
        }
        PoseStorage.x = drive.getCurrentPose().getX(DistanceUnit.INCH);
        PoseStorage.y = drive.getCurrentPose().getY(DistanceUnit.INCH);
        PoseStorage.heading = drive.getCurrentPose().getHeading(AngleUnit.DEGREES);
        // Stop everything
        drive.stopMotors();
        shooter.stopAll();
    }
}
