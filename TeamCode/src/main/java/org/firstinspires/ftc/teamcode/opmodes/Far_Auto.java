package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
@Autonomous(name = "Far Red", group = "Autonomous")
@Configurable // Panels
public class Far_Auto extends LinearOpMode {
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
    private static final double SHOOT_TIME = 0.6; // seconds for flushing balls

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
    private final Pose2D startPose = new Pose2D(DistanceUnit.INCH, 81, 8.5, AngleUnit.RADIANS, Math.toRadians(0));
    boolean suicide = false;

    double gateTimer;

    enum State{
        StartToShoot,
        Shooting,
        IntakeTimerReset,
        ToCorner,
        CornerToShoot,
        Tape3ToShoot,
        ToTape3

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
//            if(gamepad1.left_bumper){
//                suicide = true;
//            }else if (gamepad1.right_bumper){
//                suicide = false;
//            }
//
//            // Intake wait time adjustment (with debounce)
//            if(gamepad1.dpad_up && !lastDpadUp){
//                INTAKE_WAIT_TIME += 0.5;
//            }else if(gamepad1.dpad_down && !lastDpadDown){
//                INTAKE_WAIT_TIME -= 0.5;
//            }
//
//            if(gamepad1.dpad_left && !lastDpadLeft){
//                gateCycles -= 1;
//            }else if(gamepad1.dpad_right && !lastDpadRight){
//                gateCycles += 1;
//            }
//            lastDpadUp = gamepad1.dpad_up;
//            lastDpadDown = gamepad1.dpad_down;
//            lastDpadRight = gamepad1.dpad_right;
//            lastDpadLeft = gamepad1.dpad_left;
//
//
//            // Telemetry
            telemetry.addLine("INITIALIZED SUCCESSFULLY");
//            telemetry.addLine("LeftBumper Suicide / RightBumper LastTape ");
//            telemetry.addData("Suicide: ", suicide);
//            telemetry.addLine("Dpad left / right for gate cycles");
//            telemetry.addData("gate cycles: ", gateCycles);
//            telemetry.addLine("Dpad Up/Down for Lever Timer ");
//            telemetry.addData("Intake wait time: ", INTAKE_WAIT_TIME);
            telemetry.update();
//            panelsTelemetry.update(telemetry);
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
    }



    public static class Paths {
        public PathChain StartToShoot;
        public PathChain ToTape3;
        public PathChain Tape3ToShoot;
        public PathChain ToCorner;
        public PathChain CornerToShoot;

        public Paths(Follower follower) {
            StartToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(81.000, 8.500),

                                    new Pose(81.000, 27.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            ToTape3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(81.000, 27.000),
                                    new Pose(93.033, 35.250),
                                    new Pose(120.000, 35.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Tape3ToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(120.000, 35.000),

                                    new Pose(81.000, 27.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            ToCorner = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(81.000, 27.000),

                                    new Pose(135.000, 8.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            CornerToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(135.000, 8.500),
                                    new Pose(107.000, 4.200),
                                    new Pose(81.000, 25.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

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
                shooter.setTurretAngle(-90);
                shooter.setIndexerMiddle();
                shooterRunning = true;
                drive.followPathChain(paths.StartToShoot, true);
                pathTimer.reset();
                break;
            case Shooting:
                // Shoot at position 1
                shooting = true;
                shootTimer.reset();
                alignTimer.reset();
                atShot = false;
                break;
            // === TO TAPE 2 ===

            case IntakeTimerReset:
                // Wait for intake at tape 2
                IntakeTimer.reset();
                break;

            // === SECOND SHOT ===

            // === TO LEVER (first cycle) ===

            // === TO CORNER ===
            case ToCorner:
                // Shoot4toCorner: Go to corner with intake
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                drive.followPathChain(paths.ToCorner, true);
                pathTimer.reset();
                break;


            // === FINAL SHOT ===
            case CornerToShoot:
                shooter.setTurretAngle(-90);

                // CornertoShoot5: Return to final shooting position
                shooter.stopIntakeSystem();
                shooterRunning = true;
                drive.followPathChain(paths.CornerToShoot, true);
                pathTimer.reset();
                break;

            case ToTape3:
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                drive.followPathChain(paths.ToTape3, true);
                pathTimer.reset();
                break;

            case Tape3ToShoot:
                shooter.setTurretAngle(-90);
                // CornertoShoot5: Return to final shooting position
                shooter.stopIntakeSystem();
                shooterRunning = true;
                drive.followPathChain(paths.Tape3ToShoot, true);
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
                    nextState = State.ToTape3;
                }
                break;

            // === TO TAPE 3 ===
            case ToTape3:
                if(!drive.isBusy()) {
                    setPathState(State.IntakeTimerReset);
                    nextState = State.Tape3ToShoot;
                }
                break;

            case Tape3ToShoot:
                if(!drive.isBusy()) {
                    setPathState(State.Shooting);
                    nextState = State.ToCorner;
                }
                break;

            case IntakeTimerReset:
                // Immediately transition (no wait for tape positions)
                if (!drive.isBusy()) {
                    setPathState(nextState);
                }
                break;

            // === SIXTH SHOT (from tape 1) ===

            case Shooting:
                // Shoot at position 6
                if(!atShot) {
                    if(!drive.isBusy()) {
                        atShot = true;
                        shooter.stopIntakeSystem();
                        shooter.unblockShooter();
                    }
                } else {
                    limelight.update();
                    limelightTurretAutoAlign();
//                    targetShooterVelocity = updateTargetShooterVelocity();
                    if(shooting) {
                        boolean isAligned = limelight.isAlignedForShooting();
                        boolean timedOut = alignTimer.seconds() >= ALIGN_TIMEOUT;
                        boolean shooterSpeedReady = Math.abs(shooter.getShooterVelocity() - targetShooterVelocity) <= Shooter.VELOCITY_TOLERANCE;

                        if(shooterSpeedReady && (isAligned || timedOut)) {
                            shooter.unblockShooter();
                            shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                            shootTimer.reset();
                            shooting = false;
                            targetShooterVelocity = updateTargetShooterVelocity();
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
                    nextState = State.ToCorner;
                }
                break;



        }
    }
}
