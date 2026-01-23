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

/**
 * Odometry-Based Turret Tracking Test
 * 
 * Stripped-down opmode for testing turret tracking using ONLY odometry:
 * - NO Limelight (eliminates ~5-10ms per loop)
 * - NO Shooter subsystem overhead
 * - Direct turret motor control with position-based P control
 * - Configurable Kp and FF via Panels
 * 
 * Use this to benchmark turret tracking efficiency vs MainTeleOp.
 */
@Configurable
@TeleOp(name = "Odometry Turret Test", group = "Tuning")
public class TurretMotorTuning extends LinearOpMode {

    // Position P Coefficient - Configurable via Panels
    public static double posKp = 0.04;        // Position-based Kp (ticks error -> power)
    public static double turnFF = -0.9;       // Turn feedforward (counteract base rotation)
    public static double maxPower = 1.0;      // Max turret power
    
    // Hardware
    private Drive drive;
    private DcMotorEx turretMotor;
    private TelemetryManager panelsTelemetry;
    
    // Alliance for goal tracking
    private boolean isRedAlliance = true;
    
    // Goal position (set based on alliance)
    private double goalX;
    private double goalY;
    
    // Loop timing
    private long lastLoopTime = 0;
    private double loopTimeMs = 0;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);  // Match MainTeleOp optimization
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        
        // Initialize Drive subsystem (includes Pinpoint odometry)
        RobotConstants.setRobot(RobotConstants.ROBOT_21171);
        drive = new Drive(hardwareMap);
        drive.setAlliance(isRedAlliance);
        
        // Initialize turret motor directly (no Shooter subsystem)
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set goal position
        goalX = isRedAlliance ? RobotConstants.GOAL_RED_X : RobotConstants.GOAL_BLUE_X;
        goalY = isRedAlliance ? RobotConstants.GOAL_RED_Y : RobotConstants.GOAL_BLUE_Y;
        
        // ==================== ALLIANCE SELECT ====================
        while (!opModeIsActive() && !isStopRequested()) {
            // Alliance selection
            if (gamepad1.b) {
                isRedAlliance = true;
                drive.setAlliance(true);
                goalX = RobotConstants.GOAL_RED_X;
                goalY = RobotConstants.GOAL_RED_Y;
            } else if (gamepad1.x) {
                isRedAlliance = false;
                drive.setAlliance(false);
                goalX = RobotConstants.GOAL_BLUE_X;
                goalY = RobotConstants.GOAL_BLUE_Y;
            }
            
            telemetry.addLine("=== ODOMETRY TURRET TEST ===");
            telemetry.addLine("Pure position-based tracking (NO Limelight)");
            telemetry.addLine("");
            telemetry.addLine("B = RED | X = BLUE");
            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addData("Pinpoint Status", drive.getPinpointStatus());
            telemetry.addLine("");
            telemetry.addData("posKp", "%.4f", posKp);
            telemetry.addData("turnFF", "%.2f", turnFF);
            telemetry.addData("maxPower", "%.2f", maxPower);
            telemetry.addLine("");
            telemetry.addLine("Press START when ready");
            telemetry.update();
            panelsTelemetry.update(telemetry);
        }
        
        waitForStart();
        lastLoopTime = System.nanoTime();
        
        // Main loop
        while (opModeIsActive()) {
            long loopStart = System.nanoTime();
            
            // ==================== ODOMETRY UPDATE ====================
            // This is the ONLY sensor read (besides turret encoder)
            drive.updateOdometry();
            
            // ==================== DRIVETRAIN ====================
            double driveInput = -gamepad1.left_stick_y;
            double strafeInput = gamepad1.left_stick_x;
            double turnInput = -gamepad1.right_stick_x;
            drive.mecanumDrive(driveInput, strafeInput, turnInput, 1.0);
            
            // ==================== POSITION-BASED TURRET CONTROL ====================
            // Calculate target angle from odometry
            double targetAngle = drive.calculateTurretAngleToGoal(goalX, goalY);
            double targetTicks = angleToTurretTicks(targetAngle);
            double currentTicks = turretMotor.getCurrentPosition();
            double errorTicks = targetTicks - currentTicks;
            
            // P control with turn feedforward
            double pTerm = errorTicks * posKp;
            double ffTerm = turnInput * turnFF;
            double turretOutput = pTerm + ffTerm;
            turretOutput = Math.max(-maxPower, Math.min(maxPower, turretOutput));
            
            turretMotor.setPower(turretOutput);
            
            // ==================== LOOP TIMING ====================
            long loopEnd = System.nanoTime();
            loopTimeMs = (loopEnd - loopStart) / 1_000_000.0;
            
            // ==================== TELEMETRY ====================
            telemetry.addLine("=== ODOMETRY ===");
            telemetry.addData("X", "%.1f in", drive.getOdometryX() / 25.4);
            telemetry.addData("Y", "%.1f in", drive.getOdometryY() / 25.4);
            telemetry.addData("Heading", "%.1f°", drive.getOdometryHeading());
            telemetry.addLine("");
            
            telemetry.addLine("=== TURRET ===");
            telemetry.addData("Target Angle", "%.1f°", targetAngle);
            telemetry.addData("Target Ticks", "%.0f", targetTicks);
            telemetry.addData("Current Ticks", "%.0f", currentTicks);
            telemetry.addData("Error", "%.0f ticks", errorTicks);
            telemetry.addData("P term", "%.3f", pTerm);
            telemetry.addData("FF term", "%.3f", ffTerm);
            telemetry.addData("Output", "%.3f", turretOutput);
            telemetry.addLine("");
            
            telemetry.addLine("=== PERFORMANCE ===");
            telemetry.addData("Loop Time", "%.1f ms", loopTimeMs);
            telemetry.addData("Loop Rate", "%.0f Hz", 1000.0 / loopTimeMs);
            telemetry.addLine("");
            
            telemetry.addLine("=== TUNING (Panels) ===");
            telemetry.addData("posKp", "%.4f", posKp);
            telemetry.addData("turnFF", "%.2f", turnFF);
            telemetry.addData("maxPower", "%.2f", maxPower);
            
            telemetry.update();
            panelsTelemetry.update(telemetry);
        }
        
        // Stop motors
        turretMotor.setPower(0);
        drive.stopMotors();
    }
    
    /**
     * Convert turret angle (degrees) to motor ticks.
     * Copied from Shooter to avoid Shooter subsystem dependency.
     */
    private double angleToTurretTicks(double angleDegrees) {
        double normalizedAngle = angleDegrees;
        while (normalizedAngle < 0) normalizedAngle += 360;
        while (normalizedAngle >= 360) normalizedAngle -= 360;
        
        if (normalizedAngle < RobotConstants.TURRET_MIN_ANGLE) {
            normalizedAngle = RobotConstants.TURRET_MIN_ANGLE;
        } else if (normalizedAngle > RobotConstants.TURRET_MAX_ANGLE) {
            normalizedAngle = RobotConstants.TURRET_MAX_ANGLE;
        }
        
        return (normalizedAngle - RobotConstants.TURRET_MIN_ANGLE) * RobotConstants.TURRET_TICKS_PER_DEGREE;
    }
}
