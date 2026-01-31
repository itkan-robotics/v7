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

@Autonomous(name = "Far Auto Blue", group = "Autonomous")
@Configurable
public class FarAuto_BLUE extends LinearOpMode {

    private TelemetryManager panelsTelemetry;

    private Limelight limelight;
    private Shooter shooter;
    private Follower follower;

    private Paths paths;

    // Timer for shooting phases
    private ElapsedTime shootTimer = new ElapsedTime();
    private static final double SHOOT_TIME = 0.5;

    // Timer for turret alignment timeout
    private ElapsedTime alignTimer = new ElapsedTime();
    private static final double ALIGN_TIMEOUT = 0.5;
    
    // Timer for initial turret acquisition
    private ElapsedTime turretAcquireTimer = new ElapsedTime();
    private static final double TURRET_ACQUIRE_TIMEOUT = 1.5;

    boolean atShot = false;

    // Timer to prevent race condition when starting new paths
    private ElapsedTime pathTimer = new ElapsedTime();
    private static final double MIN_PATH_TIME = 0.1;

    // Flag for shooter control
    private boolean shooterRunning = false;
    private boolean shooting = false;

    // Turret target angle
    private double targetTurretAngle = 45;
    
    // Flag for StartShot - has turret acquired the tag?
    private boolean turretAcquiredTarget = false;

    // Target shooter velocity
    private double targetShooterVelocity = RobotConstants.DEFAULT_TARGET_SHOOTER_VELOCITY;

    // Starting pose - far side start position (mirrored from RED: 144 - 80 = 64)
    private final Pose2D startPose = new Pose2D(DistanceUnit.INCH, 64, 8.350, AngleUnit.RADIANS, Math.toRadians(180));

    // Configuration options
    boolean skipTape3 = false;
    int cycleCount = 4;

    enum State {
        // Initial shot from start
        StartShot,
        Shooting,
        
        // Path to shot point (when skipping tape3)
        StartToShotPoint,
        
        // Tape 3 intake (skippable)
        ToTape3,
        Tape3ToShoot,
        
        // Human player intake
        ToHuman,
        HumanToShoot,
        
        // Cycling states
        ToCyclePoint1,
        ToCyclePoint2,
        CycleToShot,
        
        // End
        Leave
    }

    State state;
    State nextState;
    int remainingCycles;

    @Override
    public void runOpMode() throws InterruptedException {
        // === INIT ===
        RobotConstants.setRobot(RobotConstants.ROBOT_21171);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        limelight = new Limelight(hardwareMap);
        shooter = new Shooter(hardwareMap);

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(
                startPose.getX(DistanceUnit.INCH),
                startPose.getY(DistanceUnit.INCH),
                startPose.getHeading(AngleUnit.RADIANS)
        ));
        paths = new Paths(follower);

        limelight.start();
        limelight.switchPipeline(0);

        // Start shooter's limelight thread for visual tracking (BLUE alliance)
        shooter.startLimelightThread(false);

        // === INIT LOOP - Configure settings ===
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;

        while (!isStarted() && !isStopRequested()) {
            limelight.update();

            // Skip Tape 3 toggle
            if (gamepad1.left_bumper) {
                skipTape3 = true;
            } else if (gamepad1.right_bumper) {
                skipTape3 = false;
            }

            // Cycle count adjustment
            if (gamepad1.dpad_up && !lastDpadUp) {
                cycleCount++;
            } else if (gamepad1.dpad_down && !lastDpadDown) {
                cycleCount = Math.max(0, cycleCount - 1);
            }
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;

            // Telemetry
            telemetry.addLine("=== FAR AUTO BLUE ===");
            telemetry.addLine("LB: Skip Tape3 / RB: Include Tape3");
            telemetry.addData("Skip Tape 3", skipTape3);
            telemetry.addLine("Dpad Up/Down for cycle count");
            telemetry.addData("Cycle Count", cycleCount);
            telemetry.update();
            panelsTelemetry.update(telemetry);
        }

        // === INIT SERVOS RIGHT AFTER START ===
        shooter.initServos();
        shooter.blockShooter();

        remainingCycles = cycleCount;
        ElapsedTime telemetryTimer = new ElapsedTime();
        
        setPathState(State.StartShot);

        // === MAIN LOOP ===
        while (opModeIsActive()) {
            // CRITICAL: follower.update() must run as fast as possible
            follower.update();

            // State machine update (includes turret control)
            autonomousPathUpdate();

            // Update shooter motor (bang-bang control)
            shooter.updateShooter(shooterRunning, targetShooterVelocity);

            // === THROTTLED OPERATIONS (every 100ms) ===
            if (telemetryTimer.milliseconds() > 100) {
                telemetryTimer.reset();

                // LED CONTROL (BLUE uses tag ID 20)
                boolean shooterReady = Math.abs(shooter.getShooterVelocity() - targetShooterVelocity) <= RobotConstants.VELOCITY_TOLERANCE;
                shooter.updateLightServo(
                        shooterRunning,
                        shooterReady,
                        !follower.isBusy(),
                        !shooting && atShot,
                        shooter.getDetectedAprilTagId(false) == 20,
                        shooter.isTurretAligned(false),
                        state == State.Shooting,
                        shooter.hasThreeBalls()
                );

                // Telemetry
                telemetry.addData("State", state);
                telemetry.addData("Remaining Cycles", remainingCycles);
                telemetry.addData("Turret Angle", targetTurretAngle);
                telemetry.addData("Shooter TPS", shooter.getShooterVelocity());
                telemetry.update();
            }
        }

        // Cleanup
        shooter.stopAll();
    }

    public static class Paths {
        public PathChain StartToShotPoint;
        public PathChain ToTape3;
        public PathChain Tape3ToShoot;
        public PathChain ToHuman;
        public PathChain HumanToShoot;
        public PathChain ToCyclePoint1;
        public PathChain ToCyclePoint2;
        public PathChain CycleToShot;
        public PathChain Leave;

        public Paths(Follower follower) {
            // All X coordinates mirrored: 144 - X
            // Headings mirrored: 180 - angle
            
            // Path from starting position to first shot point (used when skipping tape3)
            // RED: (80, 8.35) → (97.5, 10.5) | BLUE: (64, 8.35) → (46.5, 10.5)
            StartToShotPoint = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(64.000, 8.350),
                                    new Pose(46.500, 10.500)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
            
            // RED: (80, 8.35) → (80, 35.864) → (125, 36) | BLUE: (64, 8.35) → (64, 35.864) → (19, 36)
            ToTape3 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(64.000, 8.350),
                                    new Pose(64.000, 35.864),
                                    new Pose(19.000, 36.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            // RED: (125, 36) → (97.5, 10.5) heading 0° | BLUE: (19, 36) → (46.5, 10.5) heading 180°
            Tape3ToShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(19.000, 36.000),
                                    new Pose(46.500, 10.500)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            // RED: (97.5, 10.5) → (137.311, 8.874) | BLUE: (46.5, 10.5) → (6.689, 8.874)
            ToHuman = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(46.500, 10.500),
                                    new Pose(6.689, 8.874)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            // RED: (137.311, 8.874) → (97.5, 10.5) | BLUE: (6.689, 8.874) → (46.5, 10.5)
            HumanToShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(6.689, 8.874),
                                    new Pose(46.500, 10.500)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            // RED: (97.5, 10.5) → (128.5, 10.951) | BLUE: (46.5, 10.5) → (15.5, 10.951)
            ToCyclePoint1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(46.500, 10.500),
                                    new Pose(15.500, 10.951)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

//            // RED: (128.5, 10.951) → (128.5, 49.107) heading 45° | BLUE: (15.5, 10.951) → (15.5, 49.107) heading 135°
//            ToCyclePoint2 = follower.pathBuilder()
//                    .addPath(
//                            new BezierLine(
//                                    new Pose(15.500, 10.951),
//                                    new Pose(15.500, 49.107)
//                            )
//                    )
//                    .setConstantHeadingInterpolation(Math.toRadians(135))
//                    .build();

            // Updated to match ToCyclePoint1 end → shot point (mirrored from RED)
            CycleToShot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(15.500, 10.951),
                                    new Pose(46.500, 10.500)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            // RED: (97.5, 10.5) → (104.553, 18.301) | BLUE: (46.5, 10.5) → (39.447, 18.301)
            Leave = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(46.500, 10.500),
                                    new Pose(39.447, 18.301)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }

    /**
     * Sets the path state and starts following the corresponding path
     */
    public void setPathState(State state) {
        this.state = state;
        if(state == State.ToCyclePoint2) {
            this.state = State.CycleToShot;
        }
        switch (this.state) {
            case StartShot:
                // Initial shot - first move turret to hardcoded angle, then visual track
                // Starting at (64, 8.35), heading 180°, goal at (6, 143.5)
                // Delta: (-58, 135.15), world angle ≈ 113°
                // Turret angle = 113° - 180° = -67° → normalized = 293°
                targetTurretAngle = 293;
                turretAcquiredTarget = false;
                shooterRunning = true;
                shooting = true;
                shootTimer.reset();
                alignTimer.reset();
                turretAcquireTimer.reset();
                atShot = false;
                break;

            case Shooting:
                // Generic shooting state
                shooter.stopIntakeSystem();
                shooter.unblockShooter();
                shooting = true;
                shootTimer.reset();
                alignTimer.reset();
                atShot = false;
                break;

            case StartToShotPoint:
                // Go from starting position to shot point (when skipping tape3)
                // Path heading ≈ 173° (tangent from 64,8.35 to 46.5,10.5)
                // Angle to goal ≈ 107°, turret = 107 - 173 = -66° → 294°
                targetTurretAngle = 294;
                shooterRunning = true;
                follower.followPath(paths.StartToShotPoint, true);
                pathTimer.reset();
                break;

            case ToTape3:
                // Go to tape 3 for intake
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(-RobotConstants.INTAKE_POWER);
                follower.followPath(paths.ToTape3, true);
                pathTimer.reset();
                break;

            case Tape3ToShoot:
                // Return to shot point from tape 3 (intake stays on)
                // Shot point (46.5, 10.5), heading 180°, goal (6, 143.5)
                // Delta: (-40.5, 133), world angle ≈ 107°
                // Turret angle = 107° - 180° = -73° → 287°
                targetTurretAngle = 287;
                // Keep intake running while headed to shot
                shooterRunning = true;
                follower.followPath(paths.Tape3ToShoot, true);
                pathTimer.reset();
                break;

            case ToHuman:
                // Go to human player for intake
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(-RobotConstants.INTAKE_POWER);
                follower.followPath(paths.ToHuman, true);
                pathTimer.reset();
                break;

            case HumanToShoot:
                // Return to shot point from human player (intake stays on)
                // Reversed tangent from (6.689, 8.874) → heading ≈ 182°
                // Turret = 107° - 182° = -75° → 285°
                targetTurretAngle = 285;
                // Keep intake running while headed to shot
                shooterRunning = true;
                follower.followPath(paths.HumanToShoot, true);
                pathTimer.reset();
                break;

            case ToCyclePoint1:
                // Start of cycle - go to first cycle point
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(-RobotConstants.INTAKE_POWER);
                follower.followPath(paths.ToCyclePoint1, true);
                pathTimer.reset();
                break;

            case ToCyclePoint2:
                //Currently Cut out
                // Continue to second cycle point (intake still running)
                follower.followPath(paths.ToCyclePoint2, true);
                pathTimer.reset();
                break;

            case CycleToShot:
                // Return to shot point from cycle
                // Path from (15.5, 10.951) → (46.5, 10.5), reversed tangent
                // Path direction ≈ -1°, reversed → heading ≈ 179°
                // Shot point (46.5, 10.5) → goal (6, 143.5), world angle ≈ 107°
                // Turret = 107 - 179 = -72° → 288°
                targetTurretAngle = 288;
                shooter.stopIntakeSystem();
                shooterRunning = true;
                follower.followPath(paths.CycleToShot, true);
                pathTimer.reset();
                break;

            case Leave:
                // Final state - stop everything and leave
                shooter.blockShooter();
                shooterRunning = false;
                shooter.stopIntakeSystem();
                follower.followPath(paths.Leave, true);
                pathTimer.reset();
                break;
        }
    }

    /**
     * State machine for autonomous path following.
     */
    public void autonomousPathUpdate() {
        switch (state) {
            case StartShot:
                // Initial shot from start position
                // First move turret to hardcoded angle, then switch to visual once tag detected
                if (!turretAcquiredTarget) {
                    // Use position control until we see the tag or timeout
                    shooter.pointTurretByPosition(targetTurretAngle);
                    
                    // Check if we can see the correct tag (20 for BLUE) or timeout
                    boolean tagSeen = shooter.getDetectedAprilTagId(false) == 20;
                    boolean acquireTimedOut = turretAcquireTimer.seconds() >= TURRET_ACQUIRE_TIMEOUT;
                    
                    if (tagSeen || acquireTimedOut) {
                        turretAcquiredTarget = true;
                        atShot = true;
                        targetShooterVelocity = shooter.getTargetShooterTPS();
                        shooter.unblockShooter();
                        alignTimer.reset();
                    }
                } else {
                    // Tag acquired (or timed out) - use visual tracking if tag visible, otherwise position
                    if (shooter.getDetectedAprilTagId(false) == 20) {
                        shooter.pointTurretVisual(false);
                    } else {
                        shooter.pointTurretByPosition(targetTurretAngle);
                    }
                    
                    if (shooting) {
                        boolean isAligned = shooter.isTurretAligned(false);
                        boolean timedOut = alignTimer.seconds() >= ALIGN_TIMEOUT;
                        boolean shooterSpeedReady = Math.abs(shooter.getShooterVelocity() - targetShooterVelocity) <= RobotConstants.VELOCITY_TOLERANCE;

                        if (shooterSpeedReady && (isAligned || timedOut)) {
                            shooter.unblockShooter();
                            shooter.runIntakeSystem(-RobotConstants.INTAKE_POWER);
                            shootTimer.reset();
                            shooting = false;
                        }
                    } else {
                        if (shootTimer.seconds() >= SHOOT_TIME) {
                            shooter.blockShooter();
                            shooter.stopIntakeSystem();
                            // After initial shot, go to Tape3 or shot point based on config
                            if (skipTape3) {
                                setPathState(State.StartToShotPoint);
                            } else {
                                setPathState(State.ToTape3);
                            }
                        }
                    }
                }
                break;

            case StartToShotPoint:
                // Following path from start to shot point (when skipping tape3)
                shooter.pointTurretByPosition(targetTurretAngle);
                if (!follower.isBusy()) {
                    setPathState(State.Shooting);
                    nextState = State.ToHuman;
                }
                break;

            case ToTape3:
                // Following path to tape 3
                if (!follower.isBusy()) {
                    setPathState(State.Tape3ToShoot);
                }
                break;

            case Tape3ToShoot:
                // Following path back to shot point
                shooter.pointTurretByPosition(targetTurretAngle);
                if (!follower.isBusy()) {
                    setPathState(State.Shooting);
                    nextState = State.ToHuman;
                }
                break;

            case ToHuman:
                // Following path to human player
                if (!follower.isBusy()) {
                    setPathState(State.HumanToShoot);
                }
                break;

            case HumanToShoot:
                // Following path back to shot point
                shooter.pointTurretByPosition(targetTurretAngle);
                if (!follower.isBusy()) {
                    remainingCycles--;
                    setPathState(State.Shooting);
                    // After human shot, cycle back to ToHuman if cycles remain
                    if (remainingCycles > 0) {
                        nextState = State.ToHuman;
                    } else {
                        nextState = State.Leave;
                    }
                }
                break;

            case Shooting:
                // At shot position - use visual tracking
                shooter.pointTurretVisual(false);  // false = BLUE alliance
                if (!atShot) {
                    if (!follower.isBusy()) {
                        atShot = true;
                        targetShooterVelocity = shooter.getTargetShooterTPS();
                        shooter.stopIntakeSystem();
                        shooter.unblockShooter();
                    }
                } else {
                    if (shooting) {
                        boolean isAligned = shooter.isTurretAligned(false);
                        boolean timedOut = alignTimer.seconds() >= ALIGN_TIMEOUT;
                        boolean shooterSpeedReady = Math.abs(shooter.getShooterVelocity() - targetShooterVelocity) <= RobotConstants.VELOCITY_TOLERANCE;

                        if (shooterSpeedReady && (isAligned || timedOut)) {
                            shooter.unblockShooter();
                            shooter.runIntakeSystem(-RobotConstants.INTAKE_POWER);
                            shootTimer.reset();
                            shooting = false;
                        }
                    } else {
                        if (shootTimer.seconds() >= SHOOT_TIME) {
                            shooter.blockShooter();
                            shooter.stopIntakeSystem();
                            setPathState(nextState);
                        }
                    }
                }
                break;

            case ToCyclePoint1:
                // Following path to first cycle point
                if (!follower.isBusy()) {
                    setPathState(State.ToCyclePoint2);
                }
                break;

            case ToCyclePoint2:
                // Following path to second cycle point
                if (!follower.isBusy()) {
                    setPathState(State.CycleToShot);
                }
                break;

            case CycleToShot:
                // Following path back to shot point from cycle
                shooter.pointTurretByPosition(targetTurretAngle);
                if (!follower.isBusy()) {
                    remainingCycles--;
                    setPathState(State.Shooting);
                    // Determine next state after this shot
                    if (remainingCycles > 0) {
                        nextState = State.ToCyclePoint1;
                    } else {
                        nextState = State.Leave;
                    }
                }
                break;

            case Leave:
                // Final state - just wait for path to complete
                if (!follower.isBusy()) {
                    shooter.stopAll();
                }
                break;
        }
    }
}

