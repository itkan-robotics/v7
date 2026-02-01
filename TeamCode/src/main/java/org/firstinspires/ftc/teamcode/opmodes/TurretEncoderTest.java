package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Test OpMode to control turret with soft limits.
 * 
 * Controls:
 *   Dpad Left  = Move turret negative direction
 *   Dpad Right = Move turret positive direction
 * 
 * Stops at limits (10 and 950 ticks).
 */
@TeleOp(name = "Turret Encoder Test", group = "Test")
public class TurretEncoderTest extends LinearOpMode {

    private DcMotorEx turretMotor;
    
    // Soft limits
    private static final double MIN_TICKS = 10;
    private static final double MAX_TICKS = 950;

    @Override
    public void runOpMode() {
        // Initialize turret motor
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret_motor");
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(0);

        telemetry.addLine("=== TURRET ENCODER TEST ===");
        telemetry.addLine("");
        telemetry.addLine("Dpad Left  = Move negative");
        telemetry.addLine("Dpad Right = Move positive");
        telemetry.addLine("");
        telemetry.addData("Min Limit", "%.0f", MIN_TICKS);
        telemetry.addData("Max Limit", "%.0f", MAX_TICKS);
        telemetry.addLine("");
        telemetry.addData("Status", "Press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double encoderPos = turretMotor.getCurrentPosition();
            
//            // Check limits
//            boolean atMinLimit = encoderPos <= MIN_TICKS;
//            boolean atMaxLimit = encoderPos >= MAX_TICKS;
//
//            // Manual control with soft limits
//            double power = 0;
//
//            if (gamepad1.dpad_left && !atMinLimit) {
//                // Negative direction - block if at min limit
//                    power = -1;
//
//            } else if (gamepad1.dpad_right && !atMaxLimit) {
//                // Positive direction - block if at max limit
//                    power = 1;
//            } else if(atMaxLimit){
//                power = -0.15;
//            }else if(atMinLimit){
//                power = 0.15;
//            }else{
//                power = 0;
//            }
//
//            turretMotor.setPower(power);

            // Telemetry
            telemetry.addLine("=== TURRET ENCODER TEST ===");
            telemetry.addLine("");
            telemetry.addData("Encoder Position", "%.0f ticks", encoderPos);
//            telemetry.addData("Motor Power", "%.2f", power);
//            telemetry.addLine("");
//            telemetry.addData("Min Limit", "%.0f %s", MIN_TICKS, atMinLimit ? "[BLOCKED]" : "");
//            telemetry.addData("Max Limit", "%.0f %s", MAX_TICKS, atMaxLimit ? "[BLOCKED]" : "");
            telemetry.update();
        }
        
        turretMotor.setPower(0);
    }
}
