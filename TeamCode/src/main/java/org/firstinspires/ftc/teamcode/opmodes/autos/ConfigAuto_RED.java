package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

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

    private Limelight limelight;
    private Shooter shooter;
    private Follower follower; // Pedro Pathing follower

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

    // Turret target angle - must be updated continuously with P-control
    private double targetTurretAngle = 315;  // Default angle for RED

    // AprilTag ID detected during init (21, 22, or 23)
    private int detectedTagId = 21;

    // Target shooter velocity (will be updated from limelight)
    private double targetShooterVelocity = RobotConstants.DEFAULT_TARGET_SHOOTER_VELOCITY;
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
        RobotConstants.setRobot(RobotConstants.ROBOT_21171);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        limelight = new Limelight(hardwareMap);
        shooter = new Shooter(hardwareMap);

        // Initialize Pedro Pathing follower directly
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(
                startPose.getX(DistanceUnit.INCH),
                startPose.getY(DistanceUnit.INCH),
                startPose.getHeading(AngleUnit.RADIANS)
        ));
        paths = new Paths(follower);


        limelight.start();
        limelight.switchPipeline(0);
        
        // Start shooter's limelight thread for visual tracking (RED alliance)
        shooter.startLimelightThread(true);

        // === INIT LOOP - Configure settings ===
        // NOTE: Do not move any hardware during init - turret assumed in correct starting position
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;
        boolean lastDpadLeft = false;
        boolean lastDpadRight = false;

        while (!isStarted() && !isStopRequested()) {
            limelight.update();
            // NOTE: Do not move hardware during init loop

            // Suicide toggle
            if (gamepad1.left_bumper) {
                suicide = true;
            } else if (gamepad1.right_bumper) {
                suicide = false;
            }

            // Intake wait time adjustment (with debounce)
            if (gamepad1.dpad_up && !lastDpadUp) {
                INTAKE_WAIT_TIME += 0.5;
            } else if (gamepad1.dpad_down && !lastDpadDown) {
                INTAKE_WAIT_TIME -= 0.5;
            }

            if (gamepad1.dpad_left && !lastDpadLeft) {
                gateCycles -= 1;
            } else if (gamepad1.dpad_right && !lastDpadRight) {
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

        // === INIT SERVOS RIGHT AFTER START ===
        shooter.initServos();
        shooter.blockShooter();
        
        IntakeTimer = new ElapsedTime();
        ElapsedTime telemetryTimer = new ElapsedTime();  // Throttle telemetry updates
        setPathState(State.StartToShoot);
        
        // === MAIN LOOP ===
        while (opModeIsActive()) {
            // CRITICAL: follower.update() must run as fast as possible
            follower.update();
            
            // State machine update (includes turret control)
            autonomousPathUpdate();

            // Update shooter motor (bang-bang control)
            shooter.updateShooter(shooterRunning, targetShooterVelocity);
            
            // === THROTTLED OPERATIONS (every 100ms) ===
            // These are slow and don't need to run every loop
            if (telemetryTimer.milliseconds() > 100) {
                telemetryTimer.reset();
                
                // LED CONTROL (calls expensive hasThreeBalls)
                boolean shooterReady = Math.abs(shooter.getShooterVelocity() - targetShooterVelocity) <= RobotConstants.VELOCITY_TOLERANCE;
                shooter.updateLightServo(
                    shooterRunning,
                    shooterReady,
                    !follower.isBusy(),
                    !shooting && atShot,
                    shooter.getDetectedAprilTagId(true) == 24,
                    shooter.isTurretAligned(true),
                    state == State.Shooting,
                    shooter.hasThreeBalls()
                );

                // Telemetry (very slow - only update periodically)
                telemetry.addData("State", state);
                telemetry.addData("Turret Angle", targetTurretAngle);
                telemetry.addData("Shooter TPS", shooter.getShooterVelocity());
                telemetry.update();
            }
        }
        
        // Cleanup
        shooter.stopAll();
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
                            new BezierLine(new Pose(125, 39.5), new Pose(83.000, 75.000))
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
                            new BezierLine(new Pose(83.000, 75.000), new Pose(95, 70))
                    )
                    .setTangentHeadingInterpolation()
                    .build();



        }
    }

    public double calculateTurretAngleToGoalAuto() {
        double heading = follower.getHeading();
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();

        // Calculate delta from robot center to goal
        double deltaX = 138.0 * 24 - x - RobotConstants.TURRET_CENTER_OFFSET * Math.cos(heading);
        double deltaY = 143.5 * 24 - y - RobotConstants.TURRET_CENTER_OFFSET * Math.sin(heading);

        // Field angle from robot to goal (0° = positive X axis, increases counter-clockwise)
        double fieldAngleToGoal = Math.toDegrees(Math.atan2(deltaY, deltaX));

        // Turret angle = angle to goal relative to robot's front (intake direction)
        double turretAngle = fieldAngleToGoal - heading; // add 90 to convert robot angle to turret angle

        // Normalize to 0-360 range
        while (turretAngle < 0) turretAngle += 360;
        while (turretAngle >= 360) turretAngle -= 360;

        return turretAngle;
    }

    // TURRET SERVO CODE REMOVED - Now using motor instead
    // TODO: Implement turret motor auto-align using PID
    private boolean limelightTurretAutoAlign() {
        // TURRET SERVO CODE REMOVED - Now using motor instead
        // TODO: Implement turret motor PID control based on limelight tx
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
                // Calculated: pos(95,88), heading -109°, goal(138,143.5) → turret 161°
                targetTurretAngle = 161;
                shooterRunning = true;
                follower.followPath(paths.StartToShot, true);
                pathTimer.reset();
                break;
            case Shooting:
                // Shoot - switch to visual tracking for fine alignment
                shooter.stopIntakeSystem();
                shooter.unblockShooter();
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
                follower.followPath(paths.Shoot1ToTape2, true);
                pathTimer.reset();
                shooter.runIntakeSystem(-RobotConstants.INTAKE_POWER);  // Negative for intake
                break;
            case IntakeTimerReset:
                // Wait for intake at tape 2
                IntakeTimer.reset();
                break;

            // === SECOND SHOT ===
            case GoToShooting2:
                // Tape2ToShoot2: Return to shooting position 2
                // Calculated: pos(86,74), heading -23°, goal(138,143.5) → turret 76°
                targetTurretAngle = 76;
                shooterRunning = true;
                follower.followPath(paths.Tape2ToSHoot2, true);
                pathTimer.reset();
                atShot = false;
                break;

            // === TO LEVER (first cycle) ===
            case ToLever:
                // Shoot2ToLever: Close blocker, turn off shooter, run intake
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(-RobotConstants.INTAKE_POWER);  // Negative for intake
                follower.followPath(paths.Shoot2ToLever, true);
                pathTimer.reset();
                break;

            case WaitAtLever:
                // Wait for intake at lever
                IntakeTimer.reset();
                break;

            // === THIRD SHOT ===
            case ShootLever:
                // LeverToShoot3: Return from lever to shooting position
                // Calculated: pos(86,74), heading -10°, goal(138,143.5) → turret 64°
                targetTurretAngle = 64;
                shooterRunning = true;
                follower.followPath(paths.LeverToShoot3, true);
                pathTimer.reset();
                break;

            // === TO TAPE 1 ===
            case ToTape1:
                // Shoot3ToTape1: Go to tape 1 with intake
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(-RobotConstants.INTAKE_POWER);  // Negative for intake
                follower.followPath(paths.Shoot3ToTape1, true);
                pathTimer.reset();
                break;

            // === FOURTH SHOT (from tape 1) ===
            case Tape1Shoot:
                // tape1ToShoot4: Go to shooting position
                // Calculated: pos(98,85), heading -2°, goal(138,143.5) → turret 58°
                targetTurretAngle = 58;
                shooter.stopIntakeSystem();
                shooterRunning = true;
                follower.followPath(paths.tape1ToShoot4, true);
                pathTimer.reset();
                break;

            // === TO CORNER ===
            case ToCorner:
                // Shoot4toCorner: Go to corner with intake
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(-RobotConstants.INTAKE_POWER);  // Negative for intake
                follower.followPath(paths.Shoot4toCorner, true);
                pathTimer.reset();
                break;


            // === FIFTH SHOT (from corner) ===
            case CornerToShoot:
                // CornertoShoot5: Return to final shooting position
                // Calculated: pos(86.5,73.5), heading -41°, goal(138,143.5) → turret 95°
                targetTurretAngle = 95;
                shooter.stopIntakeSystem();
                shooterRunning = true;
                follower.followPath(paths.CornertoShoot5, true);
                pathTimer.reset();
                break;

            case End:
                follower.followPath(paths.End, true);
                break;

            case Suicide:
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(-1);  // Negative for intake
                follower.followPath(paths.Suicide, true);
                pathTimer.reset();
                break;

            case SuicideToShoot:
                // Return to shooting position from suicide
                // Calculated: pos(87,75), heading -52°, goal(138,143.5) → turret 106°
                targetTurretAngle = 106;
                shooter.stopIntakeSystem();
                shooterRunning = true;
                follower.followPath(paths.SuicideToShoot, true);
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
                shooter.pointTurretByPosition(targetTurretAngle);
                if (!follower.isBusy()) {
                    setPathState(State.Shooting);
                    nextState = State.ToTape2;
                }
                break;

            // === TO TAPE 2 ===
            case ToTape2:
                // Following Shoot1ToTape2
                if (!follower.isBusy()) {
                    setPathState(State.IntakeTimerReset);
                    nextState = State.GoToShooting2;
                }
                break;

            // === SECOND SHOT ===
            case GoToShooting2:
                // Following Tape2ToShoot2
                shooter.pointTurretByPosition(targetTurretAngle);
                if (!follower.isBusy()) {
                    setPathState(State.Shooting);
                    nextState = State.ToLever;
                }
                break;


            // === TO LEVER (first cycle) ===
            case ToLever:
                // Following Shoot2ToLever
                if (!follower.isBusy()) {
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
                shooter.pointTurretByPosition(targetTurretAngle);
                if(pathTimer.seconds() > 0.5) {
                    shooter.stopIntakeSystem();
                }
                if (!follower.isBusy()) {
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
                if (!follower.isBusy()) {
                    setPathState(State.IntakeTimerReset);
                    nextState = State.Tape1Shoot;
                }
                break;
            case IntakeTimerReset:
                // Immediately transition (no wait for tape positions)
                if (!follower.isBusy()) {
                    setPathState(nextState);
                }
                break;

            // === SIXTH SHOT (from tape 1) ===
            case Tape1Shoot:
                // Following tape1ToShoot4
                shooter.pointTurretByPosition(targetTurretAngle);
                if (!follower.isBusy()) {
                    setPathState(State.Shooting);
                    if(suicide){
                        nextState = State.Suicide;
                    }else {
                        nextState = State.ToCorner;
                    }
                }
                break;
            case Shooting:
                // At shot position - use visual tracking
                shooter.pointTurretVisual(true);  // true = RED alliance
                if(!atShot) {
                    if(!follower.isBusy()) {
                        atShot = true;
                        // Get target velocity once when robot stops at shot point
                        targetShooterVelocity = shooter.getTargetShooterTPS();
                        shooter.stopIntakeSystem();
                        shooter.unblockShooter();
                    }
                } else {
                    if(shooting) {
                        boolean isAligned = shooter.isTurretAligned(true);  // true = RED alliance
                        boolean timedOut = alignTimer.seconds() >= ALIGN_TIMEOUT;
                        boolean shooterSpeedReady = Math.abs(shooter.getShooterVelocity() - targetShooterVelocity) <= RobotConstants.VELOCITY_TOLERANCE;

                        if(shooterSpeedReady && (isAligned || timedOut)) {
                            shooter.unblockShooter();
                            shooter.runIntakeSystem(-(RobotConstants.INTAKE_POWER - 0.1));  // Negative to feed balls
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
                if (!follower.isBusy()) {
                    setPathState(State.IntakeTimerReset);
                    nextState = State.CornerToShoot;

                }
                break;

            // === FINAL SHOT ===
            case CornerToShoot:
                // Following CornertoShoot5
                shooter.pointTurretByPosition(targetTurretAngle);
                if (!follower.isBusy()) {
                    setPathState(State.Shooting);
                    nextState = State.End;
                }
                break;

            case Suicide:
                if (!follower.isBusy()) {
                    setPathState(State.SuicideToShoot);
                }
                break;

            case SuicideToShoot:
                shooter.pointTurretByPosition(targetTurretAngle);
                if (!follower.isBusy()) {
                    setPathState(State.Shooting);
                    nextState = State.End;
                }
                break;


        }
    }
}