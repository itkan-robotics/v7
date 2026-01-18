package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name="Meet3BlueNear")
public class Meet3BlueNear extends LinearOpMode {  // ADD extends LinearOpMode
    
    // ADD THIS CONSTANT (adjust based on your wheel diameter and motor CPR)
    static final double COUNTS_PER_MOTOR_REV = 537.7;  // e.g., for GoBILDA 312 RPM
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // adjust if geared
    static final double WHEEL_DIAMETER_INCHES = 3.77953;  // e.g., 96mm wheels
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                          (WHEEL_DIAMETER_INCHES * 3.1415);
    
    // Drivetrain motors
    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx backLeft = null;
    private DcMotorEx backRight = null;

    // Subsystem motors
    private DcMotorEx intake = null;
    private DcMotorEx transfer = null;
    private DcMotorEx sha = null;
    private DcMotorEx shb = null;

    // Servos
    private Servo blocker = null;
    Limelight3A limelight;
    Servo rgb, turret, turret2;
    
    // IMU (BHI260AP)
    private IMU imu;

    // Blocker positions
    private static final double BLOCKER_BLOCK_POSITION   = 0.1;
    private static final double BLOCKER_UNBLOCK_POSITION = 0.2;

    // Shooter settings
    private double targetShooterVelocity = 1400;
    double error, currentShooterVelocity;

    private double kp = 0.5, kf = 0.045;
    double dkp = 0.02,dkf = 0.065, dkd = 0.0011;
    double targetTx = 1;
    boolean moving = false;
    boolean aligned = false;
    double shooterPower = 0.0;

    int targetId = 24;
    double SWYFT_ENCODER_RESOLUTION = 35;
    double MIN_SHOOTER_VELOCITY = 1300;
    double MAX_SHOOTER_VELOCITY = 2200;

    private boolean SHOOTING = false;
    boolean blue = true;
    boolean driveMode = true;
    
    double currentvelocity = 0.0;
    
    
    enum ShooterState {
        IDLE,
        RAMPING,
        FIRE
    }
    
    ShooterState state = ShooterState.IDLE;
    
    @Override
    public void runOpMode() {
        initializeHardware();
        waitForStart();
        blocker.setPosition(BLOCKER_BLOCK_POSITION);
        if (opModeIsActive()) {
            // STEP 1 & 2: Auto-align and fire
            //   blocker.setPosition(BLOCKER_BLOCK_POSITION);
            //   turnNew(3075);// 360 DEGREES
            //   turnNew(1570); //180 DEGREES
            //   turnNew(743.75); // 45 degrees
            /// testing stuff here 
            
           // turnNew(180);
            
             strafeRight(18,1);
             turnNew(-80);
            driveBackward(30,0.7);
            fire(1400);
            turnNew(150);
            intake(54);
             sleep(100);
             driveForward(27,0.7);
             turnNew(-155);
            fire(1450);
            turnNew(80);
            driveBackward(21,0.7);
            turnNew(105);
            intake(60);
            driveForward(26,0.7);
            turnNew(-120);
            driveForward(22,1);
            turnNew(-70);
            fire(1450);
            strafeLeft(22,1);
            sleep(100);
            // turnNew(743.75);
            //  //turnToAngle(45);
            //  driveBackward(20,1);
            // turnNew(1486);
            // //  turnToAngle(85);
            //  intake(60);
            
            // strafeLeft(17,1);
             // // turnToAngle(-45);
            // driveBackward(28,1);
            // fire(1400);
            //turnNew(1250);
            // turnToAngle(130);
            // intake(50);
            // sleep(100);
            // driveForward(33,0.7);
            // turnNew(-1500);
            // //turnToAngle(-133);
            // // fire(1450);
            // turnNew(743.75);
            //  //turnToAngle(45);
            //  driveBackward(20,1);
            // turnNew(1486);
            // //  turnToAngle(85);
            //  intake(60);
            
            
            
             
            // driveForward(30,1);
            // sleep(100);
            // turnToAngle(-135);
            // strafeLeft(32,0.9);
            // fire(1550);
            // strafeRight(30,0.9);
        }
    }
    private void turnNew(double deg){
        deg = -deg;
         resetDriveEncoders();
         double countsPerDeg = 3240/360;
        int flT =  -(int)(deg * countsPerDeg);
        int frT =  (int)(deg * countsPerDeg);
        int blT =  -(int)(deg * countsPerDeg);
        int brT =  (int)(deg * countsPerDeg);
         frontLeft.setPower(-0.5 * Math.signum(deg));
          frontRight.setPower(0.5 * Math.signum(deg));
        backLeft.setPower(-0.5 * Math.signum(deg));
        backRight.setPower(0.5 * Math.signum(deg));
        //ElapsedTime timer = new ElapsedTime();
        double da,db,dc,dd;
        if(deg > 0){
        da = frontLeft.getCurrentPosition() - flT;
        db = frT - frontRight.getCurrentPosition() ;
        dc = backLeft.getCurrentPosition() - blT;
        dd = brT - backRight.getCurrentPosition();
        }
        else{
        da = flT - frontLeft.getCurrentPosition() ;
        db = frontRight.getCurrentPosition()  - frT;
        dc = blT - backLeft.getCurrentPosition() ;
        dd = backRight.getCurrentPosition() - brT;
        }
        double avg = Math.abs(flT-frontLeft.getCurrentPosition()) +
        Math.abs(frT-frontRight.getCurrentPosition()) + 
        Math.abs(blT-backLeft.getCurrentPosition()) + Math.abs(brT-backRight.getCurrentPosition());
        avg = (da+db+dc+dd);
        avg /=4;
        //  telemetry.addData("flT", flT);
        //      telemetry.addData("frT", frT);
        //      telemetry.addData("brT", brT);
        //      telemetry.addData("blT", blT);
             telemetry.update();
        while (avg > 300)
         {
             if(avg < 800){
        frontLeft.setPower(-0.2 * Math.signum(deg));
          frontRight.setPower(0.2 * Math.signum(deg));
        backLeft.setPower(-0.2 * Math.signum(deg));
        backRight.setPower(0.2 * Math.signum(deg));
             }
             telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
             telemetry.addData("backLeft", backLeft.getCurrentPosition());
             telemetry.addData("frontRight", frontRight.getCurrentPosition());
             telemetry.addData("backRight", backRight.getCurrentPosition());
             telemetry.update();
        if(deg > 0){
        da = frontLeft.getCurrentPosition() - flT;
        db = frT - frontRight.getCurrentPosition() ;
        dc = backLeft.getCurrentPosition() - blT;
        dd = brT - backRight.getCurrentPosition();
        }
        else{
        da = flT - frontLeft.getCurrentPosition() ;
        db = frontRight.getCurrentPosition()  - frT;
        dc = blT - backLeft.getCurrentPosition() ;
        dd = backRight.getCurrentPosition() - brT;
        }
        avg = (da+db+dc+dd);
        avg /=4;             
         }
        stopDriveMotors();
    }
    
    private void encoderDrive(double speed,
                              double flInches, double frInches,
                              double blInches, double brInches,
                              double timeoutS) {
        resetDriveEncoders();
        int flT =  (int)(flInches * COUNTS_PER_INCH);
        int frT =  (int)(frInches * COUNTS_PER_INCH);
        int blT =  (int)(blInches * COUNTS_PER_INCH);
        int brT =  (int)(brInches * COUNTS_PER_INCH);
        frontLeft.setPower(Math.abs(speed) * Math.signum(flInches));
        frontRight.setPower(Math.abs(speed) * Math.signum(frInches));
        backLeft.setPower(Math.abs(speed) * Math.signum(blInches) );
        backRight.setPower(Math.abs(speed) * Math.signum(brInches));
       
        //ElapsedTime timer = new ElapsedTime();
        double avg = Math.abs(flT-frontLeft.getCurrentPosition()) +  Math.abs(frT-frontRight.getCurrentPosition()) + Math.abs(blT-backLeft.getCurrentPosition()) + Math.abs(brT-backRight.getCurrentPosition());
        avg /=4;
        //  telemetry.addData("flT", flT);
        //      telemetry.addData("frT", frT);
        //      telemetry.addData("brT", brT);
        //      telemetry.addData("blT", blT);
             telemetry.update();
        while (avg > 300)
         {
     avg = Math.abs(flT - frontLeft.getCurrentPosition()) +  Math.abs(frT-frontRight.getCurrentPosition()) + Math.abs(blT-backLeft.getCurrentPosition()) + Math.abs(brT-backRight.getCurrentPosition());
        avg /=4;             
         }
        stopDriveMotors();
    }

    private void intake(double inches){
         intake.setPower(1);
            transfer.setPower(1);
            sleep(200);
            ElapsedTime timer = new ElapsedTime();
            while(timer.seconds() < (inches/35.0)){
                double error = inches/50.0-timer.seconds();
                error = Math.max(error, 0.2);
                  frontLeft.setPower(-error);
                 frontRight.setPower(-error);
                 backLeft.setPower(-error);
                 backRight.setPower(-error);
            }
            transfer.setPower(0);
    }
    private void turnToAngle(double targetAngle) {
        imu.resetYaw();
        double error = 0;
        double kP = 0.02; // proportional gain
        double tolerance = 1; // degree
        double maxPower = 0.5;
          double currentAngle = getHeading();
             error = targetAngle - currentAngle;
            while(Math.abs(error) > tolerance) {
                currentAngle = getHeading();
                telemetry.addData("currentAngle",currentAngle);
                telemetry.update();
             error = targetAngle - currentAngle;
            double turnPower = error * kP;

            // Limit power
            turnPower = Math.max(-maxPower, Math.min(maxPower, turnPower));

            // Apply turning power
            setTurnPower(turnPower);
            }
        stopDriveMotors();
    }
     private double getHeading() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }
    private void fire(int velo) {
        ElapsedTime timer = new ElapsedTime();
        boolean openGate = false;
        double time = (1.0*velo) / 800.0 +1.7 ;
         while(timer.seconds() < time){
            turret2.setPosition(0.2);
            turret.setPosition(0.8);
              transfer.setPower(0.8);
                intake.setPower(1);
             if(Math.abs(sha.getVelocity()) < velo){
                      sha.setPower(1);
                     shb.setPower(1);
             }
             else{
                 sha.setPower(0);
                 shb.setPower(0);
                 openGate = true;
             }
             if(openGate){
                  
                  blocker.setPosition(BLOCKER_UNBLOCK_POSITION);
             }
             else{
                 blocker.setPosition(BLOCKER_BLOCK_POSITION);
             }
         }
         sha.setPower(0);
        shb.setPower(0);
        transfer.setPower(0);
        intake.setPower(0);
         blocker.setPosition(BLOCKER_BLOCK_POSITION);
    }
    public void toIdle() {
        sha.setPower(0.0);
        shb.setPower(0.0);
        blocker.setPosition(BLOCKER_BLOCK_POSITION);
        intake.setPower(0);
        transfer.setPower(0);
    }
    
    public void driveForward(double inches, double speed) {
        encoderDrive(speed, inches, inches, inches, inches, 5.0);
    }

    public void driveBackward(double inches, double speed) {
        encoderDrive(speed, -inches, -inches, -inches, -inches, 5.0);
    }

    public void strafeLeft(double inches, double speed) {
        encoderDrive(speed, -inches, inches, inches, -inches, 5.0);
    }

    public void strafeRight(double inches, double speed) {
        encoderDrive(speed, inches, -inches, -inches, inches, 5.0);
    }

    public void setRunToPositionMode() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void stopDriveMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetDriveEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initializeHardware() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        sha = hardwareMap.get(DcMotorEx.class, "sha");
        shb = hardwareMap.get(DcMotorEx.class, "shb");

        intake.setDirection(DcMotor.Direction.FORWARD);
        transfer.setDirection(DcMotor.Direction.FORWARD);
        sha.setDirection(DcMotor.Direction.FORWARD);
        shb.setDirection(DcMotor.Direction.REVERSE);

        sha.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sha.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        blocker = hardwareMap.get(Servo.class, "blocker");
        rgb = hardwareMap.get(Servo.class, "rgb");
        turret = hardwareMap.get(Servo.class, "turret");
        turret2 = hardwareMap.get(Servo.class, "turret2");
        turret.setDirection(Servo.Direction.FORWARD);
        turret2.setDirection(Servo.Direction.REVERSE);
        

        blocker.setPosition(BLOCKER_BLOCK_POSITION);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(5);
        limelight.start();
          limelight.pipelineSwitch(0);
        limelight.pipelineSwitch(0);
         imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        // Reset heading
        imu.resetYaw();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
     private void setTurnPower(double power) {
        frontLeft.setPower(-power);
        backLeft.setPower(-power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }
}
