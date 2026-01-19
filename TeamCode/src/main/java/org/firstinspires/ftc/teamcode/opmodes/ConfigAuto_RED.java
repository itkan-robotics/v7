package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Autonomous(name = "Config Auto Red", group = "Autonomous")
@Configurable // Panels
public class ConfigAuto_RED extends LinearOpMode {
    //ctrl + f new Pose, new Pose(144-
    //ctrl f Math.toRadians(, Math.toRadians(-180-
    //multiply turrent heading by -1
    ElapsedTime IntakeTimer;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private Drive drive;
    private Limelight limelight;
    private Shooter shooter;

    private Paths paths; // Paths defined in the Paths class

    // Timer for shooting phases
    private ElapsedTime shootTimer = new ElapsedTime();
    private static final double SHOOT_TIME = 0.5; // seconds for flushing balls

    // Timer for turret alignment timeout
    private ElapsedTime alignTimer = new ElapsedTime();
    private static final double ALIGN_TIMEOUT = 0.5; // max seconds to wait for alignment before shooting anyway

    boolean atShot = false;

    double INTAKE_WAIT_TIME = 2.0; // seconds to wait for intake at lever

    // Timer to prevent race condition when starting new paths
    private ElapsedTime pathTimer = new ElapsedTime();
    private static final double MIN_PATH_TIME = 0.1; // minimum time before checking if path is done

    // Flag for shooter control - must be updated continuously for bang-bang
    private boolean shooterRunning = false;
    private boolean shooting = false; // Track if shooting sequence has started

    // AprilTag ID detected during init (21, 22, or 23)
    private int detectedTagId = 21;

    // Target shooter velocity (will be updated from limelight)
    private double targetShooterVelocity = Shooter.DEFAULT_TARGET_SHOOTER_VELOCITY;
    // Starting pose - MUST match the beginning of Path1!
    private final Pose2D startPose = new Pose2D(DistanceUnit.INCH, 111, 135, AngleUnit.RADIANS, Math.toRadians(-90));
    boolean suicide = false;

    double gateTimer;

    enum State{
        StartToShoot,
        Shooting,
        ToTape2,
        IntakeTimerReset,
        GoToShooting2,
        ToLever,
        WaitAtLever,
        ShootLever,
        ToTape1,
        Tape1Shoot,
        ToCorner,
        CornerToShoot,
        End,
        Suicide,
        SuicideToShoot
    }
    State state;
    State nextState;

    int gateCycles = 3;
    @Override
    public void runOpMode() throws InterruptedException {
        // === INIT ===
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        drive = new Drive(hardwareMap);
        limelight = new Limelight(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shooter.setIndexerMiddle();

        drive.setStartingPose(startPose);
        PoseStorage.savePose(startPose);  // Save initial pose for TeleOp
        PoseStorage.setAlliance(true);    // Red alliance
        paths = new Paths(drive.getFollower());


        limelight.start();
        limelight.switchPipeline(0);
        shooter.blockShooter(); // Start with blocker closed

        // === INIT LOOP - Configure settings ===
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;
        boolean lastDpadLeft = false;
        boolean lastDpadRight = false;

        while (!isStarted() && !isStopRequested()) {
            limelight.update();
            shooter.blockShooter();

            // Suicide toggle
            if(gamepad1.left_bumper){
                suicide = true;
            }else if (gamepad1.right_bumper){
                suicide = false;
            }

            // Intake wait time adjustment (with debounce)
            if(gamepad1.dpad_up && !lastDpadUp){
                INTAKE_WAIT_TIME += 0.5;
            }else if(gamepad1.dpad_down && !lastDpadDown){
                INTAKE_WAIT_TIME -= 0.5;
            }

            if(gamepad1.dpad_left && !lastDpadLeft){
                gateCycles -= 1;
            }else if(gamepad1.dpad_right && !lastDpadRight){
                gateCycles += 1;
            }
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;
            lastDpadRight = gamepad1.dpad_right;
            lastDpadLeft = gamepad1.dpad_left;


            // Telemetry
            telemetry.addLine("INITIALIZED SUCCESSFULLY");
            telemetry.addLine("LeftBumper Suicide / RightBumper LastTape ");
            telemetry.addData("Suicide: ", suicide);
            telemetry.addLine("Dpad left / right for gate cycles");
            telemetry.addData("gate cycles: ", gateCycles);
            telemetry.addLine("Dpad Up/Down for Lever Timer ");
            telemetry.addData("Intake wait time: ", INTAKE_WAIT_TIME);
            telemetry.update();
            panelsTelemetry.update(telemetry);
        }

        IntakeTimer = new ElapsedTime();
        setPathState(State.StartToShoot);
        // === MAIN LOOP ===
        while (opModeIsActive()) {
            drive.update(); // Update Pedro Pathing
            //limelight.update(); // Update Limelight
            autonomousPathUpdate(); // Update autonomous state machine

            // Update shooter target velocity from limelight
//            targetShooterVelocity = updateTargetShooterVelocity();
            telemetry.addData("limelight tx:  ", limelight.getTx());
            telemetry.addData("Actual Speed of Shooter", shooter.getShooterVelocity());
            telemetry.addData("Target Speed of Shooter", targetShooterVelocity);
            shooter.updateShooter(shooterRunning, targetShooterVelocity);

            panelsTelemetry.update(telemetry);
        }
        PoseStorage.x = drive.getCurrentPose().getX(DistanceUnit.INCH);
        PoseStorage.y = drive.getCurrentPose().getY(DistanceUnit.INCH);
        PoseStorage.heading = drive.getCurrentPose().getHeading(AngleUnit.DEGREES);
    }


    public static class Paths {

        public PathChain StartToShot;
        public PathChain Shoot1ToTape2;
        public PathChain Tape2ToSHoot2;
        public PathChain Shoot2ToLever;          // Combined intake path to lever
        public PathChain LeverToShoot3;          // Return from lever to shooting position
        public PathChain Shoot3ToTape1;
        public PathChain tape1ToShoot4;
        public PathChain Shoot4toCorner;
        public PathChain CornertoShoot5;
        public PathChain End;
        public PathChain Suicide;
        public PathChain SuicideToShoot;



        public Paths(Follower follower) {
            StartToShot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(111.000, 135.000), new Pose(95.000, 88.000))
                    )
//                    .setConstantHeadingInterpolation(Math.toRadians(-100))
//
//                    .addPath(
//                            new BezierLine(new Pose(106.000, 122.000), new Pose(95.000, 88.000))
//                    )
                    .setTangentHeadingInterpolation()
                    .build();


            Shoot1ToTape2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(95.000, 88.000),
                                    new Pose(95.000, 63.000),
                                    new Pose(119.000, 60.000)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(-13))
                    .build();

            Tape2ToSHoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(119.000, 60), new Pose(86.000, 74))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
            // Intake run path - starts where Path3 ends
            Shoot2ToLever = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(86.000, 74), new Pose(106.250, 63.25)) //64
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-13), Math.toRadians(20))

                    .addPath(
                            new BezierLine(new Pose(106.250, 64), new Pose(135,65))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(20))
                    .build();

            // Return from lever to shooting position
            LeverToShoot3 = follower

                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(135, 65), new Pose(86.000, 74.000))
                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(-12))
//
//                    .addPath(
//                            new BezierLine(new Pose(120.000, 65.000), new Pose(87.000, 72.000))
//                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            // Intake along upper path - starts where Path5 ends
            Shoot3ToTape1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(86.000, 74.000), new Pose(127.000, 84))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            // To shooting position from Path6
            tape1ToShoot4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(127.000, 84), new Pose(98, 85.000))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            // Intake to far corner - starts where Path7 ends
            Shoot4toCorner = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(98, 85.000), new Pose(98, 65))
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierCurve(new Pose(98, 65),
                                    new Pose(98, 45),
                                    new Pose(125, 39.5))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            // Return from far corner to shooting - starts where Path10 ends
            CornertoShoot5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(125, 39.5), new Pose(86.500, 73.500))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Suicide = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(98, 85.000), new Pose(131, 18))
                    )
                    .setTangentHeadingInterpolation()
                    .build();
            SuicideToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(131.000, 18.000), new Pose(87.000, 75.000))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            End = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(86.500, 73.500), new Pose(95, 70))
                    )
                    .setTangentHeadingInterpolation()
                    .build();



        }
    }

    /**
     * Update target shooter velocity from limelight
     * @return Target velocity in ticks per second
     */
    private double updateTargetShooterVelocity() {
        if (limelight.hasTarget()) {
            return limelight.getTargetShooterTPS();
        } else {
            return Shooter.DEFAULT_TARGET_SHOOTER_VELOCITY;
        }
    }

    private boolean limelightTurretAutoAlign() {
        if (!limelight.hasTarget()) {
            return false;
        }

        int tagId = limelight.getAprilTagId();
        if (tagId != 24) {
            return false;
        }

        double tx = limelight.getTx();
        double targetOffset = limelight.calculateTargetOffset();
        double error = tx - targetOffset;

        if (Math.abs(error) > Shooter.LIMELIGHT_TOLERANCE) {
            double currentTurretAngle = shooter.getTurretAngle();
            double turretAdjustment = -error * Shooter.LIMELIGHT_KP;
            double newTurretAngle = currentTurretAngle + turretAdjustment;
            shooter.setTurretAngle(newTurretAngle);
            return true;
        }

        return false;
    }

    /**
     * Sets the path state and starts following the corresponding path
     *
     * @param state the path state to transition to
     */
    public void setPathState(State state) {
        this.state = state;
        switch (state) {
            // === FIRST SHOT ===
            case StartToShoot:
                // StartToShot: Go to first shooting position
                shooter.setTurretAngle(-18);
                shooter.setIndexerMiddle();
                shooterRunning = true;
                drive.followPathChain(paths.StartToShot, true);
                pathTimer.reset();
                break;
            case Shooting:
                // Shoot at position 1
                shooter.stopIntakeSystem();
                shooter.unblockShooter();
                targetShooterVelocity = updateTargetShooterVelocity();
                shooting = true;
                shootTimer.reset();
                alignTimer.reset();
                atShot = false;

                break;
            // === TO TAPE 2 ===
            case ToTape2:
                // Shoot1ToTape2: Go to tape 2 with intake
                shooter.blockShooter();
                shooterRunning = false;
                drive.followPathChain(paths.Shoot1ToTape2, true);
                pathTimer.reset();
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                break;
            case IntakeTimerReset:
                // Wait for intake at tape 2
                IntakeTimer.reset();
                break;

            // === SECOND SHOT ===
            case GoToShooting2:
                // Tape2ToShoot2: Return to shooting position 2
                shooter.setTurretAngle(-74);
                shooterRunning = true;
                drive.followPathChain(paths.Tape2ToSHoot2, true);
                pathTimer.reset();
                atShot = false;
                break;

            // === TO LEVER (first cycle) ===
            case ToLever:
                // Shoot2ToLever: Close blocker, turn off shooter, run intake
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                drive.followPathChain(paths.Shoot2ToLever, true);
                pathTimer.reset();
                break;

            case WaitAtLever:
                // Wait for intake at lever
                IntakeTimer.reset();
                break;

            // === THIRD SHOT ===
            case ShootLever:
                // LeverToShoot3: Set turret to -75, stop intake, turn on shooter
                shooter.setTurretAngle(-80);
                shooterRunning = true;
                drive.followPathChain(paths.LeverToShoot3, true);
                pathTimer.reset();
                break;

            // === TO TAPE 1 ===
            case ToTape1:
                // Shoot3ToTape1: Go to tape 1 with intake
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                drive.followPathChain(paths.Shoot3ToTape1, true);
                pathTimer.reset();
                break;

            // === SIXTH SHOT (from tape 1) ===
            case Tape1Shoot:
                // tape1ToShoot4: Go to shooting position
                shooter.setTurretAngle(-85);
                //  shooter.stopIntakeSystem();
                shooterRunning = true;
                drive.followPathChain(paths.tape1ToShoot4, true);
                pathTimer.reset();
                break;

            // === TO CORNER ===
            case ToCorner:
                // Shoot4toCorner: Go to corner with intake
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                drive.followPathChain(paths.Shoot4toCorner, true);
                pathTimer.reset();
                break;


            // === FINAL SHOT ===
            case CornerToShoot:
                shooter.setTurretAngle(-63);

                // CornertoShoot5: Return to final shooting position
                shooter.stopIntakeSystem();
                shooterRunning = true;
                drive.followPathChain(paths.CornertoShoot5, true);
                pathTimer.reset();
                break;

            case End:
                drive.followPathChain(paths.End, true);
                break;

            case Suicide:
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(1);
                drive.followPathChain(paths.Suicide, true);
                pathTimer.reset();
                break;

            case SuicideToShoot:
                shooter.setTurretAngle(-52);
                // CornertoShoot5: Return to final shooting position
                shooter.stopIntakeSystem();
                shooterRunning = true;
                drive.followPathChain(paths.SuicideToShoot, true);
                pathTimer.reset();
                break;
        }
    }

    /**
     * State machine for autonomous path following.
     * Checks if the current path is complete and transitions to the next state.
     */
    public void autonomousPathUpdate() {
        switch (state) {
            // === FIRST SHOT ===
            case StartToShoot:
                // Following StartToShot to first shooting position
                if (!drive.isBusy()) {
                    setPathState(State.Shooting);
                    nextState = State.ToTape2;
                }
                break;

            // === TO TAPE 2 ===
            case ToTape2:
                // Following Shoot1ToTape2
                if (!drive.isBusy()) {
                    setPathState(State.IntakeTimerReset);
                    nextState = State.GoToShooting2;
                }
                break;

            // === SECOND SHOT ===
            case GoToShooting2:
                // Following Tape2ToShoot2
                if (!drive.isBusy()) {
                    setPathState(State.Shooting);
                    nextState = State.ToLever;
                }
                break;


            // === TO LEVER (first cycle) ===
            case ToLever:
                // Following Shoot2ToLever
                if (!drive.isBusy()) {
                    setPathState(State.WaitAtLever);
                }
                break;

            case WaitAtLever:
                // Wait for intake at lever
                if (IntakeTimer.seconds() >= INTAKE_WAIT_TIME || shooter.issintakeFull()) {
                    setPathState(State.ShootLever);
                }
                break;

            // === THIRD SHOT ===
            case ShootLever:
                // Following LeverToShoot3
                if(pathTimer.seconds() > 0.5) {
                    shooter.stopIntakeSystem();
                }
                if (!drive.isBusy()) {
                    setPathState(State.Shooting);
                    gateCycles--;  // Decrement first (this trip counts as one)
                    if(gateCycles > 0){
                        nextState = State.ToLever;  // More cycles remaining, go back to lever
                    }else{
                        nextState = State.ToTape1;  // No more cycles, continue to tape 1
                    }
                }
                break;


            // === TO TAPE 1 ===
            case ToTape1:
                // Following Shoot3ToTape1
                if (!drive.isBusy()) {
                    setPathState(State.IntakeTimerReset);
                    nextState = State.Tape1Shoot;
                }
                break;
            case IntakeTimerReset:
                // Immediately transition (no wait for tape positions)
                if (!drive.isBusy()) {
                    setPathState(nextState);
                }
                break;

            // === SIXTH SHOT (from tape 1) ===
            case Tape1Shoot:
                // Following tape1ToShoot4
                if (!drive.isBusy()) {
                    setPathState(State.Shooting);
                    if(suicide){
                        nextState = State.Suicide;
                    }else {
                        nextState = State.ToCorner;
                    }
                }
                break;
            case Shooting:
                // Shoot at position 6
                if(!atShot) {
                    if(!drive.isBusy()) {
                        atShot = true;
                        shooter.stopIntakeSystem();
                        shooter.unblockShooter();
                        //  targetShooterVelocity = updateTargetShooterVelocity();
                    }
                } else {
//                    targetShooterVelocity = updateTargetShooterVelocity();
                    if(shooting) {
                        limelight.update();
                        limelightTurretAutoAlign();
                        boolean isAligned = limelight.isAlignedForShooting();
                        boolean timedOut = alignTimer.seconds() >= ALIGN_TIMEOUT;
                        boolean shooterSpeedReady = Math.abs(shooter.getShooterVelocity() - targetShooterVelocity) <= Shooter.VELOCITY_TOLERANCE;

                        if(shooterSpeedReady && (isAligned || timedOut)) {
                            shooter.unblockShooter();
                            shooter.runIntakeSystem(Shooter.INTAKE_POWER - 0.1);
                            shootTimer.reset();
                            shooting = false;
                        }
                    } else {
                        if(shootTimer.seconds() >= SHOOT_TIME) {
                            shooter.blockShooter();
                            shooter.stopIntakeSystem();
                            setPathState(nextState);
                        }
                    }
                }
                break;

            // === TO CORNER ===
            case ToCorner:
                // Following Shoot4toCorner
                if (!drive.isBusy()) {
                    setPathState(State.IntakeTimerReset);
                    nextState = State.CornerToShoot;

                }
                break;

            // === FINAL SHOT ===
            case CornerToShoot:
                // Following CornertoShoot5
                if (!drive.isBusy()) {
                    setPathState(State.Shooting);
                    nextState = State.End;
                }
                break;

            case Suicide:
                if (!drive.isBusy()) {
                    setPathState(State.SuicideToShoot);
                }
                break;

            case SuicideToShoot:
                if (!drive.isBusy()) {
                    setPathState(State.Shooting);
                    nextState = State.End;
                }
                break;


        }
    }
}