package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "AllahuAkbar", group = "Linear Opmode")
public class TeleOpCode extends LinearOpMode {

    // Drivetrain
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    // Subsystems
    private DcMotorEx intake, transfer, shooter, shooterb;
    private Servo blocker;

    // Limelight
    private Limelight3A limelight;

    // State machine shooter control
    enum ShooterState {
        IDLE,
        RAMPING,
        FIRE
    }
    
    // Alliance enum with AprilTag IDs
    enum Alliance { 
        BLUE(20), 
        RED(24); 
        
        private final int goalApriltag; 
        
        Alliance(int id) { 
            this.goalApriltag = id; 
        } 
        
        public int getGoalApriltag() {
            return goalApriltag; 
        }
    }
    
    private ShooterState state = ShooterState.IDLE;
    private int targetVel = 0;
    private boolean flush = false;
    private Alliance matchAlliance = Alliance.BLUE;  // Default alliance
    private int goalApriltag = 20;  // Default to BLUE

    // Auto-align constants
    private final double ALIGN_TOLERANCE_DEG = 1.0;
    private final double TURN_KP = -0.01;
    private final double MIN_TURN_POWER = 0.1;

    // Blocker positions
    private static final double BLOCKER_BLOCK_POSITION   = 0.1;
    private static final double BLOCKER_UNBLOCK_POSITION = 0.2;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        blocker.setPosition(BLOCKER_UNBLOCK_POSITION);
        
        // Alliance selection loop
        while (!opModeIsActive()) {
            telemetry.addData("Alliance", matchAlliance.name());
            telemetry.addData("Goal AprilTag", matchAlliance.getGoalApriltag());
            telemetry.addLine("Press B for RED, X for BLUE");
            
            if (gamepad2.b || gamepad1.b) {
                matchAlliance = Alliance.RED;
            }
            if (gamepad2.x || gamepad1.x) {
                matchAlliance = Alliance.BLUE;
            }
            
            telemetry.update();
        }
        
        // Set the goal AprilTag based on selected alliance
        goalApriltag = matchAlliance.getGoalApriltag();
        telemetry.addData("Selected Alliance", matchAlliance.name());
        telemetry.addData("Tracking AprilTag", goalApriltag);
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            double drive  = gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;
            
            // Get Limelight data - filter for the selected alliance AprilTag
            LLResult result = limelight.getLatestResult();
            double tx = -1;
            boolean hasTarget = false;
            
            if (result != null && result.isValid()) {
                // Check if the detected AprilTag matches our goal tag
                if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                    for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                        if (fiducial.getFiducialId() == goalApriltag) {
                            hasTarget = true;
                            tx = fiducial.getTargetXDegrees();
                            break;
                        }
                    }
                } else {
                    // Fallback to getTx() if fiducial results not available
                    tx = result.getTx();
                    hasTarget = true;
                }
            }
           
            // Determine target velocity based on bumper pressed
            boolean leftBumper = gamepad1.left_bumper || gamepad2.left_bumper;
            boolean rightBumper = gamepad1.right_bumper || gamepad2.right_bumper;
            boolean shootButton = leftBumper || rightBumper;
            boolean failSafe = gamepad1.a || gamepad2.a;
            
            // Set target velocity based on which bumper
            if (leftBumper) {
               // targetVel = 1500;
                targetVel = 1600;
            } else if (rightBumper) {
                targetVel = 2100;
               // targetVel = 2000;
            }
            
            // Get current velocity
            int currentVelocity = (int) Math.abs(shooter.getVelocity());
            
            boolean aligned = false;
            double headingError = 0;
            
            // Auto-align if we have a target and shooter is active
            if (hasTarget && shootButton) {
                headingError = tx + 1; 
                if (Math.abs(headingError) > ALIGN_TOLERANCE_DEG) {
                    rotate = headingError * TURN_KP - MIN_TURN_POWER * Math.signum(headingError);
                    telemetry.addData("ROTATE: ", rotate);
                } else {
                    rotate = 0;
                    aligned = true;
                }
            }
            
            // State machine for shooter control
            switch(state) {
                case RAMPING:
                    // Behavior
                    shooter.setPower(1.0);
                    shooterb.setPower(1.0);
                    blocker.setPosition(BLOCKER_BLOCK_POSITION);  // closed
                    
                    // Exit clause - transition to FIRE when velocity reached
                    if (currentVelocity > (targetVel - 150)) {
                        state = ShooterState.FIRE;
                    }
                    break;
                    
                case FIRE:
                    // Behavior
                    blocker.setPosition(BLOCKER_UNBLOCK_POSITION); // OPEN
                    shooter.setPower(0);  // reduced power during firing
                    shooterb.setPower(0);
                    flush = true;
                    
                    // Exit clause - back to IDLE if button released
                    if (!shootButton && !failSafe) {
                        state = ShooterState.IDLE;
                    }
                    
                    // If velocity drops too much, go back to RAMPING
                    if (currentVelocity < (targetVel - 100)) {
                        state = ShooterState.RAMPING;
                    }
                    break;
                    
                case IDLE:
                    // Behavior
                    shooter.setPower(0);
                    shooterb.setPower(0);
                    blocker.setPosition(BLOCKER_BLOCK_POSITION); // CLOSED
                    
                    // Exit clause - start RAMPING if shoot button pressed
                    if (shootButton || failSafe) {
                        state = ShooterState.RAMPING;
                    }
                    break;
            }
            
            // Drivetrain control
            double fl = drive + strafe + rotate;
            double fr = drive - strafe - rotate;
            double bl = drive - strafe + rotate;
            double br = drive + strafe - rotate;

            double max = Math.max(Math.abs(fl),
                    Math.max(Math.abs(fr),
                            Math.max(Math.abs(bl), Math.abs(br))));
            if (max > 1.0) {
                fl /= max; fr /= max; bl /= max; br /= max;
            }

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // Intake/Transfer control
            if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) { 
                flush = true;
            } else if (state != ShooterState.FIRE) {
                flush = false;
            }
            
            if (flush) {
                intake.setPower(-1.0); 
                transfer.setPower(-1.0); 
            } else if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
                intake.setPower(1.0); 
                transfer.setPower(1.0); 
            } else {
                intake.setPower(0.0); 
                transfer.setPower(0.0); 
            }

            // Telemetry
            telemetry.addData("Alliance", matchAlliance.name());
            telemetry.addData("Goal AprilTag", goalApriltag);
            telemetry.addData("State", state);
            telemetry.addData("Target Velocity", targetVel);
            telemetry.addData("Current Velocity", currentVelocity);
            telemetry.addData("Aligned", aligned);
            telemetry.addData("Shooter Power", shooter.getPower());
            telemetry.addData("tx", tx);
            telemetry.addData("Has Target", hasTarget);
            telemetry.addData("HEADING ERROR", headingError);
            telemetry.update();
        }
    }

    private void initHardware() {
        // Drivetrain
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");

        // Set motor directions for mecanum drive
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set drivetrain to brake mode
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize subsystem motors
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        shooter = hardwareMap.get(DcMotorEx.class, "sha");
        shooterb = hardwareMap.get(DcMotorEx.class, "shb");

        intake.setDirection(DcMotor.Direction.REVERSE);
        transfer.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        shooterb.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Servo
        blocker = hardwareMap.get(Servo.class, "blocker");
        blocker.setPosition(BLOCKER_BLOCK_POSITION);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(10);
        limelight.start();
        limelight.pipelineSwitch(0);
    }
}
