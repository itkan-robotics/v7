package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Shooter Precision Test (21171 Lunar Robot)
 * Minimal program for testing shooter accuracy.
 * 
 * Controls:
 * - Left Trigger: Intake
 * - Right Trigger: Reverse intake
 * 
 * Features:
 * - Shooter always running at 1500 TPS
 * - Blocker always open
 */
@TeleOp(name="ShooterPrecisionTest", group="Test")
public class ShooterPrecisionTest extends LinearOpMode {

    private Shooter shooter;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);

        // Initialize 21171 lunar robot
        RobotConstants.setRobot(RobotConstants.ROBOT_21171);
        shooter = new Shooter(hardwareMap);
        shooter.initServos();

        // ==================== INIT ====================
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("=== SHOOTER PRECISION TEST ===", "");
            telemetry.addData("Robot", "21171 Lunar");
            telemetry.addLine("");
            telemetry.addData("Controls", "");
            telemetry.addData("  L Trigger", "Intake");
            telemetry.addData("  R Trigger", "Reverse");
            telemetry.addLine("");
            telemetry.addData("Status", "Press START when ready");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            // ==================== SHOOTER (ALWAYS ON) ====================
            double currentTPS = shooter.getShooterTPS();
            double targetTPS = 1500.0;
            
            // Bang-bang velocity control
            double power = (currentTPS < targetTPS) ? RobotConstants.SHOOTER_MAX_POWER : 0.0;
            shooter.setShooterPower(power);
            
            // Blocker always open
            shooter.setBlocker(false);

            // ==================== INTAKE ====================
            boolean feeding = false;
            if (gamepad1.left_trigger > 0.1) {
                shooter.runIntakeSystem(1.0);
                feeding = true;
            }

            if (gamepad1.right_trigger > 0.1) {
                shooter.runIntakeSystem(-1.0);
                feeding = true;
            }

            if (!feeding) {
                shooter.setIntakePower(-0.1);
            }

            // ==================== TELEMETRY ====================
            telemetry.addData("=== SHOOTER PRECISION TEST ===", "");
            telemetry.addData("Robot", "21171 Lunar");
            telemetry.addLine("");
            
            telemetry.addData("Shooter", "%.0f / %.0f TPS", currentTPS, targetTPS);
            telemetry.addLine("");
            
            telemetry.addData("Intake", shooter.hasThreeBalls() ? "[FULL]" : "[OK]");

            telemetry.update();
        }

        shooter.stopAll();
    }
}
