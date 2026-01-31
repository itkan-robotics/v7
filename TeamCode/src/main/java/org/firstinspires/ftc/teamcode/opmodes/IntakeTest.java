package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Test OpMode for intake system components.
 * 
 * Controls:
 * =========
 * MOTOR ONLY:
 *   A = Intake (motor only)
 *   B = Outtake (motor only)
 * 
 * MOTOR + SERVOS:
 *   X = Intake (motor + servos)
 *   Y = Outtake (motor + servos)
 * 
 * SERVOS ONLY:
 *   Left Bumper = Intake (servos only)
 *   Right Bumper = Outtake (servos only)
 * 
 * POWER ADJUSTMENT:
 *   Dpad Up = Increase power
 *   Dpad Down = Decrease power
 */
@TeleOp(name = "Intake Test", group = "Test")
@Disabled
public class IntakeTest extends LinearOpMode {

    // Hardware
    private DcMotorEx intakeMotor;
    private CRServo intakeServoL;
    private CRServo intakeServoR;

    // Power level (adjustable)
    private double power = 1.0;
    private static final double POWER_INCREMENT = 0.1;
    private static final double MIN_POWER = 0.1;
    private static final double MAX_POWER = 1.0;

    // Dpad edge detection
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void runOpMode() {
        // Initialize hardware
        intakeMotor = hardwareMap.get(DcMotorEx.class, "transfer_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeServoL = hardwareMap.get(CRServo.class, "intake_servoL");
        intakeServoR = hardwareMap.get(CRServo.class, "intake_servoR");

        telemetry.addLine("=== INTAKE TEST ===");
        telemetry.addLine("");
        telemetry.addLine("MOTOR ONLY:");
        telemetry.addLine("  A = Intake | B = Outtake");
        telemetry.addLine("");
        telemetry.addLine("MOTOR + SERVOS:");
        telemetry.addLine("  X = Intake | Y = Outtake");
        telemetry.addLine("");
        telemetry.addLine("SERVOS ONLY:");
        telemetry.addLine("  LB = Intake | RB = Outtake");
        telemetry.addLine("");
        telemetry.addLine("Dpad Up/Down = Adjust Power");
        telemetry.addLine("");
        telemetry.addData("Status", "Ready - Press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Power adjustment
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;

            if (dpadUp && !lastDpadUp) {
                power = Math.min(MAX_POWER, power + POWER_INCREMENT);
            }
            if (dpadDown && !lastDpadDown) {
                power = Math.max(MIN_POWER, power - POWER_INCREMENT);
            }
            lastDpadUp = dpadUp;
            lastDpadDown = dpadDown;

            // Track what's running for telemetry
            String activeMode = "IDLE";
            String direction = "-";

            // ========== MOTOR ONLY ==========
            if (gamepad1.a) {
                // Motor only - INTAKE
                intakeMotor.setPower(-power);
                stopServos();
                activeMode = "MOTOR ONLY";
                direction = "INTAKE";
            } else if (gamepad1.b) {
                // Motor only - OUTTAKE
                intakeMotor.setPower(power);
                stopServos();
                activeMode = "MOTOR ONLY";
                direction = "OUTTAKE";
            }
            // ========== MOTOR + SERVOS ==========
            else if (gamepad1.x) {
                // Motor + Servos - INTAKE
                intakeMotor.setPower(-power);
                intakeServoL.setPower(-power);
                intakeServoR.setPower(power);  // Opposite direction
                activeMode = "MOTOR + SERVOS";
                direction = "INTAKE";
            } else if (gamepad1.y) {
                // Motor + Servos - OUTTAKE
                intakeMotor.setPower(power);
                intakeServoL.setPower(power);
                intakeServoR.setPower(-power);  // Opposite direction
                activeMode = "MOTOR + SERVOS";
                direction = "OUTTAKE";
            }
            // ========== SERVOS ONLY ==========
            else if (gamepad1.left_bumper) {
                // Servos only - INTAKE
                intakeMotor.setPower(0);
                intakeServoL.setPower(-power);
                intakeServoR.setPower(power);  // Opposite direction
                activeMode = "SERVOS ONLY";
                direction = "INTAKE";
            } else if (gamepad1.right_bumper) {
                // Servos only - OUTTAKE
                intakeMotor.setPower(0);
                intakeServoL.setPower(power);
                intakeServoR.setPower(-power);  // Opposite direction
                activeMode = "SERVOS ONLY";
                direction = "OUTTAKE";
            }
            // ========== STOP ==========
            else {
                stopAll();
                activeMode = "IDLE";
                direction = "-";
            }

            // Telemetry
            telemetry.addLine("=== INTAKE TEST ===");
            telemetry.addLine("");
            telemetry.addData("Mode", activeMode);
            telemetry.addData("Direction", direction);
            telemetry.addData("Power", "%.1f", power);
            telemetry.addLine("");
            telemetry.addLine("--- Hardware Status ---");
            telemetry.addData("Motor Power", "%.2f", intakeMotor.getPower());
            telemetry.addData("Motor Current", "%.2f A", intakeMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("ServoL Power", "%.2f", intakeServoL.getPower());
            telemetry.addData("ServoR Power", "%.2f", intakeServoR.getPower());
            telemetry.addLine("");
            telemetry.addLine("--- Controls ---");
            telemetry.addLine("A/B = Motor Only");
            telemetry.addLine("X/Y = Motor + Servos");
            telemetry.addLine("LB/RB = Servos Only");
            telemetry.addLine("Dpad = Adjust Power");
            telemetry.update();
        }

        // Cleanup
        stopAll();
    }

    private void stopServos() {
        intakeServoL.setPower(0);
        intakeServoR.setPower(0);
    }

    private void stopAll() {
        intakeMotor.setPower(0);
        intakeServoL.setPower(0);
        intakeServoR.setPower(0);
    }
}
