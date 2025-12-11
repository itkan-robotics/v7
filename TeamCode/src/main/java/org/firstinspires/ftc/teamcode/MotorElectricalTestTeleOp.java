package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * TeleOp for testing voltage and amperage on intake and transfer motors
 * 
 * Controls:
 * A - Turn on both intake and transfer motors together
 * X - Turn on intake motor only
 * Y - Turn on transfer motor only
 * B - Stop all motors
 * 
 * Telemetry displays:
 * - Battery voltage
 * - Intake motor: voltage, current draw, power, velocity
 * - Transfer motor: voltage, current draw, power, velocity
 * - Stalling indicators (high current, low velocity)
 */
@TeleOp(name = "Motor Electrical Test", group = "Test")
public class MotorElectricalTestTeleOp extends LinearOpMode {

    private HardwareConfigAuto robot;
    private RobotFunctionsAuto robotFunctions;
    
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;
    
    private boolean intakeRunning = false;
    private boolean transferRunning = false;
    
    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        robot = new HardwareConfigAuto();
        robot.init(hardwareMap);
        robotFunctions = new RobotFunctionsAuto(robot);
        
        // Get battery voltage sensor
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        
        // Set initial states
        robotFunctions.setBlocker(true);
        robotFunctions.stopIntakeSystem();
        
        telemetry.addLine("=== Motor Electrical Test TeleOp ===");
        telemetry.addLine("A = Both motors ON");
        telemetry.addLine("X = Intake only");
        telemetry.addLine("Y = Transfer only");
        telemetry.addLine("B = Stop all");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // === MOTOR CONTROL ===
            // A - Both motors on
            if (gamepad1.a && !lastA) {
                intakeRunning = true;
                transferRunning = true;
                robot.intakeMotor.setPower(HardwareConfigAuto.INTAKE_POWER);
                robot.transferMotor.setPower(HardwareConfigAuto.TRANSFER_POWER);
            }
            
            // X - Intake only
            if (gamepad1.x && !lastX) {
                intakeRunning = true;
                transferRunning = false;
                robot.intakeMotor.setPower(HardwareConfigAuto.INTAKE_POWER);
                robot.transferMotor.setPower(0);
            }
            
            // Y - Transfer only
            if (gamepad1.y && !lastY) {
                intakeRunning = false;
                transferRunning = true;
                robot.intakeMotor.setPower(0);
                robot.transferMotor.setPower(HardwareConfigAuto.TRANSFER_POWER);
            }
            
            // B - Stop all
            if (gamepad1.b && !lastB) {
                intakeRunning = false;
                transferRunning = false;
                robotFunctions.stopIntakeSystem();
            }
            
            // Update button states
            lastA = gamepad1.a;
            lastB = gamepad1.b;
            lastX = gamepad1.x;
            lastY = gamepad1.y;
            
            // === TELEMETRY ===
            telemetry.addLine("=== Motor Electrical Test ===");
            telemetry.addLine("");
            
            // Battery voltage
            double batteryVoltage = batteryVoltageSensor.getVoltage();
            telemetry.addLine("=== BATTERY ===");
            telemetry.addData("Battery Voltage", "%.2f V", batteryVoltage);
            telemetry.addLine("");
            
            // Intake motor electrical data
            telemetry.addLine("=== INTAKE MOTOR ===");
            if (robot.intakeMotor instanceof DcMotorEx) {
                DcMotorEx intakeEx = (DcMotorEx) robot.intakeMotor;
                double intakeCurrent = intakeEx.getCurrent(CurrentUnit.AMPS);
                double intakeVoltage = intakeEx.getCurrentAlert(CurrentUnit.AMPS);
                double intakePower = intakeEx.getPower();
                double intakeVelocity = Math.abs(intakeEx.getVelocity());
                
                telemetry.addData("Power", "%.2f", intakePower);
                telemetry.addData("Current Alert", "%.2f A", intakeVoltage);
                telemetry.addData("Current Draw", "%.2f A", intakeCurrent);
                telemetry.addData("Velocity", "%.1f ticks/sec", intakeVelocity);
                
                // Stalling indicators
                boolean intakeStalling = intakeCurrent > 2.0 && intakeVelocity < 100.0 && intakeRunning;
                if (intakeStalling) {
                    telemetry.addData("STATUS", "*** STALLING ***");
                } else if (intakeRunning) {
                    telemetry.addData("STATUS", "Running");
                } else {
                    telemetry.addData("STATUS", "Stopped");
                }
            } else {
                telemetry.addData("Status", "Motor not DcMotorEx - limited data");
                telemetry.addData("Power", "%.2f", robot.intakeMotor.getPower());
            }
            telemetry.addLine("");
            
            // Transfer motor electrical data
            telemetry.addLine("=== TRANSFER MOTOR ===");
            if (robot.transferMotor instanceof DcMotorEx) {
                DcMotorEx transferEx = (DcMotorEx) robot.transferMotor;
                double transferCurrent = transferEx.getCurrent(CurrentUnit.AMPS);
                double transferVoltage = transferEx.getCurrentAlert(CurrentUnit.AMPS);
                double transferPower = transferEx.getPower();
                double transferVelocity = Math.abs(transferEx.getVelocity());
                
                telemetry.addData("Power", "%.2f", transferPower);
                telemetry.addData("Current Alert", "%.2f V", transferVoltage);
                telemetry.addData("Current Draw", "%.2f A", transferCurrent);
                telemetry.addData("Velocity", "%.1f ticks/sec", transferVelocity);
                
                // Stalling indicators
                boolean transferStalling = transferCurrent > 2.0 && transferVelocity < 100.0 && transferRunning;
                if (transferStalling) {
                    telemetry.addData("STATUS", "*** STALLING ***");
                } else if (transferRunning) {
                    telemetry.addData("STATUS", "Running");
                } else {
                    telemetry.addData("STATUS", "Stopped");
                }
            } else {
                telemetry.addData("Status", "Motor not DcMotorEx - limited data");
                telemetry.addData("Power", "%.2f", robot.transferMotor.getPower());
            }
            telemetry.addLine("");
            
            // Combined power consumption
            double totalCurrent = 0.0;
            if (robot.intakeMotor instanceof DcMotorEx) {
                totalCurrent += ((DcMotorEx) robot.intakeMotor).getCurrent(CurrentUnit.AMPS);
            }
            if (robot.transferMotor instanceof DcMotorEx) {
                totalCurrent += ((DcMotorEx) robot.transferMotor).getCurrent(CurrentUnit.AMPS);
            }
            telemetry.addLine("=== COMBINED ===");
            telemetry.addData("Total Current Draw", "%.2f A", totalCurrent);
            telemetry.addData("Power Consumption", "%.2f W", totalCurrent * batteryVoltage);
            telemetry.addLine("");
            
            // Controls reminder
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("A = Both | X = Intake | Y = Transfer | B = Stop");
            
            telemetry.update();
        }
        
        // Stop all motors on exit
        robotFunctions.stopIntakeSystem();
    }
}

