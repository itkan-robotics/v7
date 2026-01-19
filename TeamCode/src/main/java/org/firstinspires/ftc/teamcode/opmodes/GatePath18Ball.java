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

@Disabled
@Autonomous(name = "Gate Path 18 Ball", group = "Autonomous")
@Configurable // Panels
public class GatePath18Ball extends LinearOpMode {
    //ctrl + f new Pose, new Pose(144-
    //ctrl f Math.toRadians(, Math.toRadians(-180-
    //multiply turrent heading by -1
    ElapsedTime IntakeTimer;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private Drive drive;
    private Limelight limelight;
    private Shooter shooter;

    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    // Timer for shooting phases
    private ElapsedTime shootTimer = new ElapsedTime();
    private static final double SHOOT_TIME = 1.5; // seconds for flushing balls

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
    private final Pose2D startPose = new Pose2D(DistanceUnit.INCH, 128.72445019404915, 113.26261319534282, AngleUnit.RADIANS, Math.toRadians(-90));

    @Override
    public void runOpMode() throws InterruptedException {
        // === INIT ===
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        drive = new Drive(hardwareMap);
        limelight = new Limelight(hardwareMap);
        shooter = new Shooter(hardwareMap);

        drive.setStartingPose(startPose);
        paths = new Paths(drive.getFollower());
        pathState = 0;

        limelight.start();
        limelight.switchPipeline(0);
        shooter.blockShooter(); // Start with blocker closed

        telemetry.addLine("INITIALIZED SUCCESSFULLY");
        telemetry.update();
        panelsTelemetry.update(telemetry);

        telemetry.addLine("Waiting");
        telemetry.update();
        // === INIT LOOP - Detect AprilTag ===
        while (!isStarted() && !isStopRequested()) {
            limelight.update();
            shooter.blockShooter();
        }

        IntakeTimer = new ElapsedTime();

        // === START ===
        setPathState(0);

        // === MAIN LOOP ===
        while (opModeIsActive()) {
            drive.update(); // Update Pedro Pathing
            limelight.update(); // Update Limelight
            autonomousPathUpdate(); // Update autonomous state machine

            // Update shooter target velocity from limelight

            // Continuously update shooter with bang-bang control
            shooter.updateShooter(shooterRunning, targetShooterVelocity);

            // Auto-align turret while moving to shooting positions
            if (pathState == 0 || pathState == 1 ||
                    pathState == 4 || pathState == 5 ||
                    pathState == 8 || pathState == 9 ||
                    pathState == 12 || pathState == 13 ||
                    pathState == 16 || pathState == 17 ||
                    pathState == 20 || pathState == 21) {
//                shooter.redlimelightTurretAutoAlign(limelight);
            }

            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
            Pose2D currentPose = drive.getCurrentPose();
            panelsTelemetry.debug("X", currentPose.getX(DistanceUnit.INCH));
            panelsTelemetry.debug("Y", currentPose.getY(DistanceUnit.INCH));
            panelsTelemetry.debug("Heading", currentPose.getHeading(AngleUnit.RADIANS));
            panelsTelemetry.debug("SHOOTER VEL >>>>   ", shooter.getShooterVelocity());
            panelsTelemetry.debug("SHOOTER TARGET >>>> ", targetShooterVelocity);
            panelsTelemetry.debug("intake full", shooter.issintakeFull());
            panelsTelemetry.debug("Power Consumption", shooter.getPowerConsumption());
            panelsTelemetry.update(telemetry);
        }
    }


    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path10;
        public PathChain Path11;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(128.724, 113.263), new Pose(92.000, 88.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(92.000, 88.000),
                                    new Pose(92.000, 60.000),
                                    new Pose(123.000, 60.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(123.000, 60.000),
                                    new Pose(76.000, 60.000),
                                    new Pose(76.000, 75.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(76.000, 75.000),
                                    new Pose(76.000, 60.000),
                                    new Pose(115.000, 53.000),
                                    new Pose(132.264, 61.288)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(132.264, 61.288),
                                    new Pose(115.000, 53.000),
                                    new Pose(76.000, 60.000),
                                    new Pose(76.000, 75.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(76.000, 75.000),
                                    new Pose(80.290, 90.908),
                                    new Pose(122.204, 83.270)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(122.204, 83.270),
                                    new Pose(94.000, 86.000),
                                    new Pose(94.000, 86.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-104))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(94.000, 86.000),
                                    new Pose(83.000, 43.000),
                                    new Pose(112.517, 54.768),
                                    new Pose(133.382, 23.472)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(133.382, 23.472), new Pose(76.000, 75.000))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
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

    /**
     * Sets the path state and starts following the corresponding path
     *
     * @param state the path state to transition to
     */
    public void setPathState(int state) {
        pathState = state;
        switch (pathState) {
            case 0:
                shooter.setTurretAngle(-31);
                shooter.setIndexerMiddle();
                shooterRunning = true;
                drive.followPathChain(paths.Path1, true);
                pathTimer.reset();
                break;
            case 1:
                shooting = true;
                shootTimer.reset();
                break;

            // End of Path1: wait for shooter ready, then shoot
            case 2:
                shooter.blockShooter();
                shooterRunning = false;
                drive.followPathChain(paths.Path2, true);
                pathTimer.reset();
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                break;

            case 3:
                IntakeTimer.reset();
                // Shooting done after Path1, start Path2 with intake ON
                break;
            case 4:
                // Path2 done, wait for balls to fall
                shooter.setTurretAngle(-50);
                shooterRunning = true;
                drive.followPathChain(paths.Path3, true);
                pathTimer.reset();
                break;
            case 5:
                shooting = true;
                shootTimer.reset();
                break;
            case 6:
                shooter.blockShooter();
                shooterRunning = false;
                drive.followPathChain(paths.Path4, true);
                pathTimer.reset();
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                break;
            case 7:
                IntakeTimer.reset();
                shooterRunning = true;
                // Shooting done after Path1, start Path2 with intake ON
                break;
            case 8:
                shooter.stopIntakeSystem();
                shooterRunning = true;
                drive.followPathChain(paths.Path5, true);
                pathTimer.reset();
                break;

            case 9:
                shooting = true;
                shootTimer.reset();
                break;
            case 10:
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                drive.followPathChain(paths.Path4, true);
                pathTimer.reset();
                break;
            case 11:
                IntakeTimer.reset();
                // Shooting done after Path1, start Path2 with intake ON
                break;
            case 12:
                shooter.stopIntakeSystem();
                shooterRunning = true;
                drive.followPathChain(paths.Path5, true);
                pathTimer.reset();
                break;
            case 13:
                shooting = true;
                shootTimer.reset();

                break;
            case 14:
                shooter.blockShooter();
                shooterRunning = false;
                drive.followPathChain(paths.Path6, true);
                pathTimer.reset();
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                break;
            case 15:
                IntakeTimer.reset();
                // Shooting done after Path1, start Path2 with intake ON
                break;
            case 16:
                shooter.setTurretAngle(-25);

                shooter.stopIntakeSystem();
                shooterRunning = true;
                drive.followPathChain(paths.Path7, true);
                pathTimer.reset();
                break;
            case 17:
                shooting = true;
                shootTimer.reset();
                break;
            case 18:
                shooter.blockShooter();
                shooterRunning = false;
                drive.followPathChain(paths.Path10, true);
                pathTimer.reset();
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                break;
            case 19:
                IntakeTimer.reset();
                // Shooting done after Path1, start Path2 with intake ON
                break;
            case 20:
                shooterRunning = true;
                shooter.stopIntakeSystem();
                drive.followPathChain(paths.Path11, true);
                pathTimer.reset();
                break;
            case 21:
                shooting = true;
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
                if (!drive.isBusy()) {
                    setPathState(1); // Wait for shooter ready, then shoot
                }
                break;
            case 1:
                // Wait for shooter ready, then open blocker and flush balls
                if (!drive.isBusy()) {
                    limelight.update(); // Update Limelight
//                    shooter.redlimelightTurretAutoAlign(limelight);
                    if (shooting && shooter.isShooterReady(targetShooterVelocity, limelight.isAlignedForShooting())) {
                        shooter.unblockShooter();
                        shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                        shootTimer.reset();
                        shooting = false;
                        targetShooterVelocity = updateTargetShooterVelocity();
                    }
                    if (!shooting && shootTimer.seconds() >= SHOOT_TIME) {
                        shooter.blockShooter();
                        shooter.stopIntakeSystem();
                        setPathState(2); // Shooting done, go to gate
                    }
                }
                break;
            case 2:
                // Following Path2 to gate with intake
                if (!drive.isBusy()) {
                    setPathState(3); // Wait for shooter ready, then shoot
                }
                break;
            case 3:
                // Wait for balls to fall into intake
                if (!drive.isBusy() && (shooter.intakeFull() || IntakeTimer.seconds() > 2)) {
                    setPathState(4); // Wait for shooter ready, then shoot
                }
                break;
            case 4:
                // Following Path3 back to shooting position
                if (!drive.isBusy()) {
                    setPathState(5);
                }
                break;
            case 5:

                // Wait for shooter ready, then open blocker and flush balls
                if (!drive.isBusy()) {
                    limelight.update(); // Update Limelight
//                    shooter.redlimelightTurretAutoAlign(limelight);
                    if (shooting && shooter.isShooterReady(targetShooterVelocity, limelight.isAlignedForShooting())) {
                        shooter.unblockShooter();
                        shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                        shootTimer.reset();
                        shooting = false;
                    }
                    if (!shooting && shootTimer.seconds() >= SHOOT_TIME) {
                        shooter.blockShooter();
                        shooter.stopIntakeSystem();
                        setPathState(6); // Shooting done, go to gate
                    }
                }
                break;
            case 6:
                // Following Path4 to gate with intake
                if (!drive.isBusy()) {
                    setPathState(7); // Wait for shooter ready, then shoot
                }
                break;
            case 7:
                // Wait for shooter ready, then open blocker and flush balls
                if (!drive.isBusy() && (shooter.issintakeFull() || IntakeTimer.seconds() > 2.5)) {
                    setPathState(8); // Wait for shooter ready, then shoot
                }
                break;
            case 8:
                // Following Path5 back to shooting position
                if (!drive.isBusy()) {
                    setPathState(9);
                }
                break;
            case 9:
                // Wait for shooter ready, then open blocker and flush balls
                if (!drive.isBusy()) {
                    limelight.update(); // Update Limelight
//                    shooter.redlimelightTurretAutoAlign(limelight);
                    if (shooting && shooter.isShooterReady(targetShooterVelocity, limelight.isAlignedForShooting())) {
                        shooter.unblockShooter();
                        shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                        shootTimer.reset();
                        shooting = false;
                    }
                    if (!shooting && shootTimer.seconds() >= SHOOT_TIME) {
                        shooter.blockShooter();
                        shooter.stopIntakeSystem();
                        setPathState(10); // Shooting done, go to gate
                    }                  }
                break;
            case 10:
                // Following Path6 to gate with intake
                if (!drive.isBusy()) {
                    setPathState(11); // Wait for shooter ready, then shoot
                }
                break;
            case 11:

                // Wait for shooter ready, then open blocker and flush balls
                if (!drive.isBusy() && (shooter.issintakeFull() || IntakeTimer.seconds() > 1.75)) {
                    setPathState(12); // Wait for shooter ready, then shoot
                }
                break;
            case 12:
                // Following Path7 back to shooting position
                if (!drive.isBusy()) {
                    setPathState(13); // Wait for shooter ready, then shoot
                }
                break;
            case 13:
                if(!drive.isBusy()) {
                    limelight.update(); // Update Limelight
//                    shooter.redlimelightTurretAutoAlign(limelight);
                    if (shooting && shooter.isShooterReady(targetShooterVelocity, limelight.isAlignedForShooting())) {
                        shooter.unblockShooter();
                        shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                        shootTimer.reset();
                        shooting = false;
                    }
                    if (!shooting && shootTimer.seconds() >= SHOOT_TIME) {
                        shooter.blockShooter();
                        shooter.stopIntakeSystem();
                        setPathState(14); // Shooting done, go to gate
                    }
                }
                break;
            case 14:
                // Following Path4 to gate with intake
                if (!drive.isBusy()) {
                    setPathState(15); // Wait for shooter ready, then shoot
                }
                break;
            case 15:
                if(!drive.isBusy() && (shooter.issintakeFull() || IntakeTimer.seconds() > 1.75)){
                    setPathState(16); // Shooting done, go to gate
                }
                break;
            case 16:
                // Following Path5 back to shooting position
                if (!drive.isBusy()) {
                    setPathState(17);
                }
                break;
            case 17:
                if(!drive.isBusy()){
                    limelight.update(); // Update Limelight
//                    shooter.redlimelightTurretAutoAlign(limelight);
                    if (shooting && shooter.isShooterReady(targetShooterVelocity, limelight.isAlignedForShooting())) {
                        shooter.unblockShooter();
                        shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                        shootTimer.reset();
                        shooting = false;
                    }
                    if (!shooting && shootTimer.seconds() >= SHOOT_TIME) {
                        shooter.blockShooter();
                        shooter.stopIntakeSystem();
                        setPathState(18); // Shooting done, go to gate
                    }
                }
                break;
            case 18:
                // Following Path10 to gate with intake
                if (!drive.isBusy()) {
                    setPathState(19); // Wait for shooter ready, then shoot
                }
                break;
            case 19:
                if(!drive.isBusy() &&(shooter.issintakeFull() || IntakeTimer.seconds() > 1.75)){
                    setPathState(20);
                }
                break;
            case 20:
                // Following Path11 back to shooting position
                if (!drive.isBusy()) {
                    setPathState(21);
                }
                break;
            case 21:
                // Final shooting sequence
                if (!drive.isBusy()) {
                    limelight.update(); // Update Limelight
//                    shooter.redlimelightTurretAutoAlign(limelight);
                    if (shooting && shooter.isShooterReady(targetShooterVelocity, limelight.isAlignedForShooting())) {
                        shooter.unblockShooter();
                        shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                        shootTimer.reset();
                        shooting = false;
                    }
                    if (!shooting && shootTimer.seconds() >= SHOOT_TIME) {
                        shooter.blockShooter();
                        shooter.stopIntakeSystem();
                        shooterRunning = false;
                        // Autonomous complete - stay in state 21
                    }
                }
                break;

        }
    }
}
