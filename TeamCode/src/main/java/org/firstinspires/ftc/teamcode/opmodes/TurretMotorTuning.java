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
 * Turret Motor Tuning OpMode
 * 
 * Stripped down opmode for tuning turret motor PID on target ticks.
 * 
 * Features:
 * - Mecanum drive using Drive subsystem
 * - Turret PID controller targeting specified ticks
 * - Telemetry: odometry position, turret angle, target ticks, error
 */
@Configurable
@TeleOp(name = "Turret Motor Tuning", group = "Tuning")
public class TurretMotorTuning extends LinearOpMode {

    // PID Coefficients - Configurable via Panels
    public static double kP = 0.085;

    public static double kI = 0.0;

    public static double kD = 0.0;

    public static double maxPower = 1.0;
    
    // Hardware
    private Drive drive;
    private DcMotorEx turretMotor;
    private TelemetryManager panelsTelemetry;
    
    // PID state
    private double integral = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;
    
    // Alliance for goal tracking
    private boolean isRedAlliance = true;

    @Override
    public void runOpMode() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        
        // Initialize Drive subsystem
        drive = new Drive(hardwareMap);
        drive.setAlliance(isRedAlliance);
        
        // Initialize turret motor
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Reset PID state
        integral = 0.0;
        lastError = 0.0;
        lastTime = System.currentTimeMillis() / 1000.0;
        
        // Init loop - alliance selection
        while (!opModeIsActive() && !isStopRequested()) {
            if (gamepad1.b) {
                isRedAlliance = true;
                drive.setAlliance(true);
            } else if (gamepad1.x) {
                isRedAlliance = false;
                drive.setAlliance(false);
            }
            
            telemetry.addLine("=== TURRET MOTOR TUNING ===");
            telemetry.addLine("B = RED | X = BLUE");
            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addLine("");
            telemetry.addData("kP", "%.4f", kP);
            telemetry.addData("kI", "%.5f", kI);
            telemetry.addData("kD", "%.4f", kD);
            telemetry.addLine("");
            telemetry.addLine("Press START when ready");
            telemetry.update();
            panelsTelemetry.update(telemetry);
        }
        
        waitForStart();
        
        // Main loop
        boolean lastY = false;
        
        while (opModeIsActive()) {
            // Update odometry
            drive.updateOdometry();
            
            // Reset position to center field with Y button
            if (gamepad1.y && !lastY) {
                // Center field position in mm, heading 270° (facing -Y direction)
                double centerX = RobotConstants.FIELD_CENTER_X_INCHES * RobotConstants.INCHES_TO_MM;
                double centerY = RobotConstants.FIELD_CENTER_Y_INCHES * RobotConstants.INCHES_TO_MM;
                drive.setOdometryPosition(centerX, centerY, 270.0);
            }
            lastY = gamepad1.y;
            
            // Mecanum drive control
            double driveInput = -gamepad1.left_stick_y;
            double strafeInput = gamepad1.left_stick_x;
            double turnInput = gamepad1.right_stick_x;
            
            drive.mecanumDrive(driveInput, strafeInput, turnInput, 1.0);
            
            // Calculate turret angle from Drive
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
            
            // Calculate output
            double output = proportional + integralTerm + derivativeTerm;
            output = Math.max(-maxPower, Math.min(maxPower, output));
            
            // Apply power
            turretMotor.setPower(output);
            
            // Update state
            lastError = error;
            lastTime = currentTime;
            
            // Telemetry
            telemetry.addLine("=== TURRET MOTOR TUNING ===");
            telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
            telemetry.addLine("Y = Reset to center field (270°)");
            telemetry.addLine("");
            
            telemetry.addLine("=== ODOMETRY ===");
            telemetry.addData("X (mm)", "%.1f", drive.getCachedX());
            telemetry.addData("Y (mm)", "%.1f", drive.getCachedY());
            telemetry.addData("Heading (deg)", "%.1f", drive.getCachedHeading());
            telemetry.addLine("");
            
            telemetry.addLine("=== TURRET ===");
            telemetry.addData("Angle to Goal (deg)", "%.1f", turretAngleDegrees);
            telemetry.addData("Target Ticks", "%.0f", targetTicks);
            telemetry.addData("Current Ticks", "%.0f", currentTicks);
            telemetry.addData("Error (ticks)", "%.0f", error);
            telemetry.addData("Output Power", "%.3f", output);
            telemetry.addLine("");
            
            telemetry.addLine("=== PID ===");
            telemetry.addData("kP", "%.4f", kP);
            telemetry.addData("kI", "%.5f", kI);
            telemetry.addData("kD", "%.4f", kD);
            telemetry.addData("P term", "%.4f", proportional);
            telemetry.addData("I term", "%.4f", integralTerm);
            telemetry.addData("D term", "%.4f", derivativeTerm);
            
            telemetry.update();
            panelsTelemetry.update(telemetry);
        }
        
        // Stop motors
        turretMotor.setPower(0);
        drive.stopMotors();
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
