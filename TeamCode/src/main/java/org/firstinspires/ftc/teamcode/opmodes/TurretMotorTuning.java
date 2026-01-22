package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Visual Turret Tracking Tuning
 * 
 * Simplified opmode for tuning visual (limelight) turret tracking:
 * - Drivetrain control
 * - Turret constantly tracks limelight target using PD control
 * - Configurable Kp and Kd via Panels
 */
@Configurable
@TeleOp(name = "Visual Turret Tuning", group = "Tuning")
public class TurretMotorTuning extends LinearOpMode {

    // Visual PD Coefficients - Configurable via Panels
    public static double kP = 0.03;
    public static double kD = 0.01;
    public static double kFF = 0.3;  // Turn feedforward
    public static double targetTxOffset = 1.0;
    public static double maxPower = 0.5;
    
    // Hardware
    private Drive drive;
    private Shooter shooter;
    private DcMotorEx turretMotor;
    private TelemetryManager panelsTelemetry;
    
    // PD state
    private double lastError = 0.0;
    
    // Alliance for goal tracking
    private boolean isRedAlliance = true;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(5);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        
        // Initialize Drive subsystem
        drive = new Drive(hardwareMap);
        drive.setAlliance(isRedAlliance);
        
        // Initialize Shooter subsystem (for limelight)
        shooter = new Shooter(hardwareMap);
        
        // Initialize turret motor directly
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Reset PD state
        lastError = 0.0;
        
        // ==================== ALLIANCE SELECT ====================
        while (!opModeIsActive() && !isStopRequested()) {
            // Alliance selection
            if (gamepad1.b) {
                isRedAlliance = true;
                drive.setAlliance(true);
            } else if (gamepad1.x) {
                isRedAlliance = false;
                drive.setAlliance(false);
            }
            
            telemetry.addLine("=== VISUAL TURRET TUNING ===");
            telemetry.addLine("B = RED | X = BLUE");
            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("Target Tag", isRedAlliance ? 24 : 20);
            telemetry.addLine("");
            telemetry.addData("kP", "%.4f", kP);
            telemetry.addData("kD", "%.4f", kD);
            telemetry.addData("kFF (turn)", "%.2f", kFF);
            telemetry.addData("Target TX Offset", "%.2f", targetTxOffset);
            telemetry.addData("Max Power", "%.2f", maxPower);
            telemetry.addLine("");
            telemetry.addLine("Press START when ready");
            telemetry.update();
            panelsTelemetry.update(telemetry);
        }
        
        waitForStart();
        
        // Main loop
        while (opModeIsActive()) {
            // ==================== LIMELIGHT UPDATE ====================
            shooter.updateLimelightData(isRedAlliance);
            
            double tx = shooter.getLimelightTx(isRedAlliance);
            double ty = shooter.getLimelightTy();
            int detectedTagId = shooter.getDetectedAprilTagId(isRedAlliance);
            boolean hasTarget = shooter.hasLimelightTarget();
            int expectedTagId = isRedAlliance ? 24 : 20;
            boolean hasCorrectTarget = hasTarget && (detectedTagId == expectedTagId);
            
            // ==================== DRIVETRAIN ====================
            double driveInput = -gamepad1.left_stick_y;
            double strafeInput = gamepad1.left_stick_x;
            double turnInput = -gamepad1.right_stick_x;
            drive.mecanumDrive(driveInput, strafeInput, turnInput, 1.0);
            
            // ==================== VISUAL PD+FF TURRET CONTROL ====================
            double error = 0;
            double derivative = 0;
            double pTerm = 0;
            double dTerm = 0;
            double ffTerm = turnInput * kFF;  // Feedforward to counteract base rotation
            double turretOutput = 0;
            
            if (hasCorrectTarget) {
                // Calculate PD control
                error = targetTxOffset - tx;
                derivative = error - lastError;
                lastError = error;
                
                pTerm = error * kP;
                dTerm = derivative * kD;
                turretOutput = pTerm + dTerm + ffTerm;
                turretOutput = Math.max(-maxPower, Math.min(maxPower, turretOutput));
            } else {
                // No target - still apply feedforward to help acquire target while turning
                turretOutput = ffTerm;
                turretOutput = Math.max(-maxPower, Math.min(maxPower, turretOutput));
                lastError = 0;
            }
            
            turretMotor.setPower(turretOutput);
            
            // ==================== TELEMETRY ====================
            telemetry.addLine("=== LIMELIGHT ===");
            telemetry.addData("Has Target", hasTarget ? "YES" : "NO");
            telemetry.addData("Correct Tag", hasCorrectTarget ? "YES (" + detectedTagId + ")" : "NO");
            telemetry.addData("tx", "%.2f°", tx);
            telemetry.addData("ty", "%.2f°", ty);
            telemetry.addLine("");
            
            telemetry.addLine("=== TURRET PD+FF ===");
            telemetry.addData("Target TX", "%.2f°", targetTxOffset);
            telemetry.addData("Error", "%.2f°", error);
            telemetry.addData("P term", "%.4f", pTerm);
            telemetry.addData("D term", "%.4f", dTerm);
            telemetry.addData("FF term", "%.4f", ffTerm);
            telemetry.addData("Output", "%.4f", turretOutput);
            telemetry.addData("Encoder", "%.0f ticks", (double) turretMotor.getCurrentPosition());
            telemetry.addLine("");
            
            telemetry.addLine("=== TUNING VALUES ===");
            telemetry.addData("kP", "%.4f", kP);
            telemetry.addData("kD", "%.4f", kD);
            telemetry.addData("kFF", "%.2f", kFF);
            telemetry.addData("Max Power", "%.2f", maxPower);
            telemetry.addLine("");
            
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("LStick = Drive | RStick = Turn");
            telemetry.addLine("Adjust kP/kD in Panels");
            
            telemetry.update();
            panelsTelemetry.update(telemetry);
        }
        
        // Stop motors
        turretMotor.setPower(0);
        drive.stopMotors();
    }
}
