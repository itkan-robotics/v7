package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Simple test OpMode to read turret encoder position.
 * Motor is set to FLOAT so you can manually move the turret.
 */
@TeleOp(name = "Turret Encoder Test", group = "Test")
public class TurretEncoderTest extends LinearOpMode {

    private DcMotorEx turretMotor;

    @Override
    public void runOpMode() {
        // Initialize turret motor
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(0);

        telemetry.addLine("=== TURRET ENCODER TEST ===");
        telemetry.addLine("Motor set to FLOAT mode");
        telemetry.addLine("Manually move turret to read encoder");
        telemetry.addLine("");
        telemetry.addData("Status", "Press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double encoderPos = turretMotor.getCurrentPosition();

            telemetry.addLine("=== TURRET ENCODER TEST ===");
            telemetry.addLine("");
            telemetry.addData("Encoder Position", "%.0f ticks", encoderPos);
            telemetry.update();
        }
    }
}
