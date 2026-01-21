package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Full TeleOp with Turret Motor Tuning
 * 
 * Complete teleop that can:
 * - Drive around (mecanum)
 * - Intake and outtake
 * - Shoot with auto TPS or manual TPS
 * - Turret motor always locked onto goal via odometry with tunable PID
 */
@Configurable
@Disabled
@TeleOp(name = "Turret Motor Tuning", group = "Tuning")
public class TurretMotorTuning extends LinearOpMode {

    // PID Coefficients - Configurable via Panels
    public static double kP = 0.025;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double maxPower = 1.0;
    
    // Turret on-target threshold (in ticks)
    public static double TURRET_ON_TARGET_THRESHOLD = 50.0;
    
    // Hardware
    private Drive drive;
    private Shooter shooter;
    private DcMotorEx turretMotor;
    private TelemetryManager panelsTelemetry;
    
    // PID state
    private double integral = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;
    
    // Robot selection (21171 or 19564)
    private int selectedRobot = RobotConstants.ROBOT_21171;
    
    // Alliance for goal tracking
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
    private boolean lastY = false;
    
    // Feeding timing for transfer power ramp
    private long feedingStartTime = 0;
    private boolean wasFeeding = false;
    
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(5);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        
        // ==================== INITIALIZE HARDWARE FIRST ====================
        RobotConstants.setRobot(RobotConstants.ROBOT_21171);
        
        // Initialize Drive subsystem
        drive = new Drive(hardwareMap);
        drive.setAlliance(isRedAlliance);
        
        // Initialize Shooter subsystem (turret motor is handled separately here)
        shooter = new Shooter(hardwareMap);
        
        // Initialize turret motor directly (not through Shooter to avoid conflicts)
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Reset PID state
        integral = 0.0;
        lastError = 0.0;
        lastTime = System.currentTimeMillis() / 1000.0;
        
        // ==================== ROBOT & ALLIANCE SELECT ====================
        while (!opModeIsActive() && !isStopRequested()) {
            // Robot selection
            if (gamepad1.y) {
                selectedRobot = RobotConstants.ROBOT_21171;
            } else if (gamepad1.a) {
                selectedRobot = RobotConstants.ROBOT_19564;
            }
            
            // Alliance selection
            if (gamepad1.b) {
                isRedAlliance = true;
                drive.setAlliance(true);
            } else if (gamepad1.x) {
                isRedAlliance = false;
                drive.setAlliance(false);
            }
            
            telemetry.addLine("=== TURRET MOTOR TUNING TELEOP ===");
            telemetry.addLine("Y = 21171  |  A = 19564");
            telemetry.addData("Robot", selectedRobot);
            telemetry.addLine("");
            telemetry.addLine("B = RED | X = BLUE");
            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("Target Tag", isRedAlliance ? 24 : 20);
            telemetry.addLine("");
            telemetry.addData("kP", "%.4f", kP);
            telemetry.addData("kI", "%.5f", kI);
            telemetry.addData("kD", "%.4f", kD);
            telemetry.addLine("");
            telemetry.addData("Pinpoint Status", drive.getPinpointStatus());
            telemetry.addLine("Press START when ready");
            telemetry.update();
            panelsTelemetry.update(telemetry);
        }
        
        // Apply the selected robot's constants
        RobotConstants.setRobot(selectedRobot);
        
        // If robot changed from default, re-apply hardware settings
        if (selectedRobot != RobotConstants.ROBOT_21171) {
            drive.applyPinpointSettings();
            shooter.applyMotorSettings();
        }
        
        // Initialize servos (after pinpoint IMU has calibrated)
        shooter.initServos();
        
        // Set alliance
        drive.setAlliance(isRedAlliance);
        
        runtime.reset();
        waitForStart();
        
        // Main loop
        while (opModeIsActive()) {
            // ==================== UPDATE ODOMETRY ====================
            drive.updateOdometry();
            
            // ==================== UPDATE LIMELIGHT DATA ====================

            // ==================== ODOMETRY RESET ====================
            // Reset position to center field with left stick button
            boolean leftStickButton = gamepad1.left_stick_button;
            if (leftStickButton && !lastLeftStickButton) {
                double fieldCenterX = RobotConstants.FIELD_CENTER_X_INCHES * RobotConstants.INCHES_TO_MM;
                double fieldCenterY = RobotConstants.FIELD_CENTER_Y_INCHES * RobotConstants.INCHES_TO_MM;
                drive.setOdometryPosition(fieldCenterX, fieldCenterY, 270.0);
            }
            lastLeftStickButton = leftStickButton;
            
            // ==================== MECANUM DRIVETRAIN ====================
            double driveInput = -gamepad1.left_stick_y;
            double strafeInput = gamepad1.left_stick_x;
            double turnInput = -gamepad1.right_stick_x;
            
            drive.mecanumDriveWithBraking(driveInput, strafeInput, turnInput, 1.0);
            
            // ==================== TURRET PID CONTROL ====================
            // Calculate turret angle from Drive (odometry-based)
            double turretAngleDegrees = drive.calculateTurretAngleToGoal(isRedAlliance);
            
            // Get current turret position
            double currentTicks = turretMotor.getCurrentPosition();
            double targetTicks = turretAngleToTicks(turretAngleDegrees);
            
            // Calculate PID
            double currentTime = System.currentTimeMillis() / 1000.0;
            double deltaTime = currentTime - lastTime;
            if (deltaTime <= 0) {
                deltaTime = 0.02;
            }
            
            double error = targetTicks - currentTicks;
            
            // Proportional
            double proportional = kP * error;
            
            // Integral with anti-windup
            integral += error * deltaTime;
            double maxIntegral = maxPower / (kI + 0.0001);
            if (Math.abs(integral) > maxIntegral) {
                integral = Math.signum(integral) * maxIntegral;
            }
            double integralTerm = kI * integral;
            
            // Derivative
            double derivative = (error - lastError) / deltaTime;
            double derivativeTerm = kD * derivative;
            

            
            // Apply turret power (negated to correct direction)
            
            // Update PID state
            lastError = error;
            lastTime = currentTime;
            
            // Check if turret is on target
            boolean turretOnTarget = Math.abs(error) < TURRET_ON_TARGET_THRESHOLD;
            
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
            
            // A button: spin up shooter to target RPM (without feeding)
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

            double turretOutput;
            if(shootButtonPressed) {
                shooter.updateLimelightData(isRedAlliance);
                turretOutput = 0.05 * (0 - shooter.getLimelightTx(isRedAlliance));
            } else {
                // Calculate output
                turretOutput = proportional;
                turretOutput = Math.max(-maxPower, Math.min(maxPower, turretOutput));
            }

            turretMotor.setPower(turretOutput);


            if (leftBumper) {
                drive.setPositionOverride(72.0, 30.0);
                shooter.setDefaultTPSOverride(1750.0);
            } else if (rightBumper) {
                drive.setPositionOverride(72.0, 90.0);
                shooter.setDefaultTPSOverride(1550.0);
            } else {
                drive.clearPositionOverride();
                shooter.clearDefaultTPSOverride();
            }
            
            double currentTPS = shooter.getShooterTPS();
            double targetTPS;
            if (manualRPMMode) {
                targetTPS = manualTargetTPS;
            } else {
                targetTPS = shooter.getTargetShooterTPS();
            }
            
            boolean shooterReady = shooter.isShooterSpeedReady(targetTPS);
            
            // ==================== INTAKE/TRANSFER CONTROLS ====================
            boolean feeding = false;
            
            // Shooting latch logic
            if (!shootButtonPressed) {
                shootingLatched = false;
            } else {
                if (shootingLatched) {
                    // Unlatch if turret goes off target
                    if (!turretOnTarget) {
                        shootingLatched = false;
                    }
                } else {
                    // Latch when shooter ready AND turret on target
                    if (shooterReady && turretOnTarget) {
                        shootingLatched = true;
                    }
                }
            }
            
            // Unblock when shoot button pressed
            if (shootButtonPressed) {
                shooter.setBlocker(false);
            }
            
            // Feed when latched and on target
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
            
            // Shooter control when bumper pressed
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
            
            // ==================== TELEMETRY ====================
            telemetry.addLine("=== STATUS ===");
            telemetry.addData("Robot", RobotConstants.getCurrentRobot());
            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("Shooting", SHOOTING);
            telemetry.addLine("");
            
            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Mode", manualRPMMode ? "MANUAL (Dpad)" : "AUTO (Limelight)");
            telemetry.addData("Velocity", "%.0f / %.0f TPS", currentTPS, targetTPS);
            telemetry.addData("Ready", shooterReady ? "YES" : "NO");
            telemetry.addData("Latched", shootingLatched ? "YES" : "NO");
            telemetry.addLine("");
            
            telemetry.addLine("=== TURRET ===");
            telemetry.addData("Angle to Goal (deg)", "%.1f", turretAngleDegrees);
            telemetry.addData("Target Ticks", "%.0f", targetTicks);
            telemetry.addData("Current Ticks", "%.0f", currentTicks);
            telemetry.addData("Error (ticks)", "%.0f", error);
            telemetry.addData("Output Power", "%.3f", turretOutput);
            telemetry.addData("On Target", turretOnTarget ? "YES" : "NO");
            telemetry.addLine("");
            
            telemetry.addLine("=== PID ===");
            telemetry.addData("kP", "%.4f", kP);
            telemetry.addData("kI", "%.5f", kI);
            telemetry.addData("kD", "%.4f", kD);
            telemetry.addData("P term", "%.4f", proportional);
            telemetry.addData("I term", "%.4f", integralTerm);
            telemetry.addData("D term", "%.4f", derivativeTerm);
            telemetry.addLine("");
            
            telemetry.addLine("=== ODOMETRY ===");
            telemetry.addData("X (in)", "%.1f", drive.getCachedX() / 25.4);
            telemetry.addData("Y (in)", "%.1f", drive.getCachedY() / 25.4);
            telemetry.addData("Heading (deg)", "%.1f", drive.getCachedHeading());
            telemetry.addLine("");
            
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("LStick = Drive | RStick = Turn");
            telemetry.addLine("RT = Intake | LT = Outtake");
            telemetry.addLine("Bumpers = Shoot | A = Spin up");
            telemetry.addLine("Dpad = Manual TPS | Back = Climber");
            telemetry.addLine("LStick Click = Reset Odometry");
            
            telemetry.update();
            panelsTelemetry.update(telemetry);
        }
        
        // Stop motors
        turretMotor.setPower(0);
        drive.stopMotors();
        shooter.stopAll();
    }

    /**
     * Convert turret angle (degrees) to motor ticks.
     * Accounts for hardstop deadzone: 0 ticks = 5°, max ticks = 355°.
     * 
     * @param angle Turret angle in degrees (-180 to 180 from calculateTurretAngleToGoal)
     * @return Motor ticks clamped to valid range
     */
    public double turretAngleToTicks(double angle) {
        // Normalize angle to 0-360 range
        double normalizedAngle = angle;
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
}
