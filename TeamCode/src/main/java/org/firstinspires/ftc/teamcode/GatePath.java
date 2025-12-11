package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(group = "Autonomous")
@Configurable // Panels
public class GatePath extends LinearOpMode {
    //ctrl + f new Pose, new Pose(144-
    //ctrl f Math.toRadians(, Math.toRadians(-180-
    //multiply turrent heading by -1
    ElapsedTime IntakeTimer;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private HardwareConfigAuto robot;
    private RobotFunctionsAuto robotFunctions;

    // Timer for shooting phases
    private ElapsedTime shootTimer = new ElapsedTime();
    private static final double SHOOT_TIME = 1.5; // seconds for flushing balls

    // Flag for shooter control - must be updated continuously for bang-bang
    private boolean shooterRunning = false;
    private boolean shootingStarted = false; // Track if shooting sequence has started

    // AprilTag ID detected during init (21, 22, or 23)
    private int detectedTagId = 21;

    // Starting pose - MUST match the beginning of Path1!
    private final Pose startPose = new Pose(72.000, 135.000, Math.toRadians(-90));

    @Override
    public void runOpMode() throws InterruptedException {
        // === INIT ===
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        paths = new Paths(follower); // Build paths
        pathState = 0;

        try {
            robot = new HardwareConfigAuto();
            robot.init(hardwareMap);
            robotFunctions = new RobotFunctionsAuto(robot);
            robotFunctions.setBlocker(true);
            telemetry.addLine("INITIALIZED SUCCESSFULLY");
        } catch (Exception e) {
            robot = null;
            robotFunctions = null;
            telemetry.addLine("THREW ERROR");
        }

        telemetry.update();
        panelsTelemetry.update(telemetry);

        telemetry.addLine("Waiting");
        telemetry.update();
        // === INIT LOOP - Detect AprilTag ===
        while (!isStarted() && !isStopRequested()) {
        }

        IntakeTimer = new ElapsedTime();


        // === START ===
        setPathState(0);

        // === MAIN LOOP ===
        while (opModeIsActive()) {
            follower.update(); // Update Pedro Pathing
            autonomousPathUpdate(); // Update autonomous state machine

            // Continuously update shooter with bang-bang control
            robotFunctions.controlShooter(shooterRunning);
            
            // Auto-align turret while moving to shooting positions
            if (pathState == 0 || pathState == 4 || pathState == 1 || pathState == 5) {
                robotFunctions.limelightTurretAutoAlign();
            }

            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.debug("SHOOTER VEL >>>>   ", robotFunctions.getShooterTPS());
            panelsTelemetry.debug("SHOOTER TARGET >>>> ", robotFunctions.getTargetShooterTPS());
            panelsTelemetry.update(telemetry);
        }
    }


    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.000, 135.000), new Pose(96.000, 96.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(-90))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(96.000, 96.000),
                                    new Pose(100.000, 64.000),
                                    new Pose(134.000, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(30))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(132.000, 60.000),
                                    new Pose(100.000, 64.000),
                                    new Pose(96.000, 96.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(-90))
                    .build();
        }
    }

    /**
     * Run the appropriate shooting sequence based on detected tag and which set
     * @param setNumber Which set of balls (2, 3, or 4)
     */
    /**
     * Sets the path state and starts following the corresponding path
     * @param state the path state to transition to
     */
    public void setPathState(int state) {
        pathState = state;
        switch (pathState) {
            case 0:
                // Start Path1, set turret and indexer, start shooter spinning
                robotFunctions.setTurretAngle(-30);
                robotFunctions.setIndexerMiddle();
                shooterRunning = true;
                follower.followPath(paths.Path1, true);
                break;
            case 1:
                // End of Path1: wait for shooter ready, then shoot
                shootingStarted = false;
                shootTimer.reset();
                break;
            case 2:
                // Shooting done after Path1, start Path2 with intake ON
                robotFunctions.setBlocker(true);
                shooterRunning = false;
                follower.followPath(paths.Path2, true);
                robotFunctions.runIntakeSystem(HardwareConfigAuto.INTAKE_POWER);
                break;
            case 3:
                // Path2 done, wait for balls to fall
                IntakeTimer.reset();
                break;
            case 4:
                // Start Path3 back to shooting position
                robotFunctions.ejectFourth();
                shooterRunning = true;
                follower.followPath(paths.Path3, true);
                break;
            case 5:
                // End of Path3: wait for shooter ready, then shoot
                robotFunctions.stopIntakeSystem();
                shootingStarted = false;
                shootTimer.reset();
                break;
        }
    }

    /**
     * State machine for autonomous path following.
     * Checks if the current path is complete and transitions to the next state.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Following Path1 to shooting position
                if (!follower.isBusy()) {
                    setPathState(1); // Wait for shooter ready, then shoot
                }
                break;
            case 1:
                // Wait for shooter ready, then open blocker and flush balls
                if (!shootingStarted && robotFunctions.isShooterReady()) {
                    robotFunctions.setBlocker(false);
                    robotFunctions.runIntakeSystem(HardwareConfigAuto.INTAKE_POWER);
                    shootTimer.reset();
                    shootingStarted = true;
                }
                if (shootingStarted && shootTimer.seconds() >= SHOOT_TIME) {
                    robotFunctions.setBlocker(true);
                    robotFunctions.stopIntakeSystem();
                    setPathState(2); // Shooting done, go to gate
                }
                break;
            case 2:
                // Following Path2 to gate with intake
                if (!follower.isBusy()) {
                    setPathState(3); // Wait for balls to fall
                }
                break;
            case 3:
                // Wait for balls to fall into intake
                if (IntakeTimer.seconds() > 1.0) {
                    setPathState(4); // Go back to shooting position
                }
                break;
            case 4:
                // Following Path3 back to shooting position
                if (!follower.isBusy()) {
                    setPathState(5); // Wait for shooter ready, then shoot
                }
                break;
            case 5:
                // Wait for shooter ready, then open blocker and flush balls
                if (!shootingStarted && robotFunctions.isShooterReady()) {
                    robotFunctions.setBlocker(false);
                    robotFunctions.runIntakeSystem(HardwareConfigAuto.INTAKE_POWER);
                    shootTimer.reset();
                    shootingStarted = true;
                }
                if (shootingStarted && shootTimer.seconds() >= SHOOT_TIME) {
                    robotFunctions.setBlocker(true);
                    robotFunctions.stopIntakeSystem();
                    setPathState(2); // Loop back to gate
                }
                break;
        }
    }
}
