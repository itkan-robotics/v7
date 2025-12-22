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

@Autonomous(name = "Gate Path 2", group = "Autonomous")
@Configurable // Panels
@Disabled
public class GatePath2 extends LinearOpMode {
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
            targetShooterVelocity = updateTargetShooterVelocity();

            // Continuously update shooter with bang-bang control
            shooter.updateShooter(shooterRunning, targetShooterVelocity);

            // Auto-align turret while moving to shooting positions
            if (pathState == 0 || pathState == 1 || pathState == 9) {
                shooter.limelightTurretAutoAlign(limelight);
            }

            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
            Pose2D currentPose = drive.getCurrentPose();
            panelsTelemetry.debug("X", currentPose.getX(DistanceUnit.INCH));
            panelsTelemetry.debug("Y", currentPose.getY(DistanceUnit.INCH));
            panelsTelemetry.debug("Heading", currentPose.getHeading(AngleUnit.RADIANS));
            panelsTelemetry.debug("SHOOTER VEL >>>>   ", shooter.getShooterVelocity());
            panelsTelemetry.debug("SHOOTER TARGET >>>> ", targetShooterVelocity);
            panelsTelemetry.debug("intake full", shooter.intakeFull());
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

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(129.842, 112.890), new Pose(87.000, 74.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(-116))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(87.000, 74.000),
                                    new Pose(90.000, 63.000),
                                    new Pose(121.000, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-116), Math.toRadians(0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(120.528, 59.426),
                                    new Pose(97.000, 69.000),
                                    new Pose(87.000, 74.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(87.000, 74.000),
                                    new Pose(97.614, 53.651),
                                    new Pose(131.146, 62.592)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(131.146, 62.592), new Pose(87.000, 74.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
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
     * @param state the path state to transition to
     */
    public void setPathState(int state) {
        pathState = state;
        switch (pathState) {
            case 0:
                // Start Path1, set turret and indexer, start shooter spinning
                shooter.setTurretAngle(-20);
                shooter.setIndexerMiddle();
                shooterRunning = true;
                drive.followPathChain(paths.Path1, true);
                break;
            case 1:
                // End of Path1: wait for shooter ready, then shoot
                shooting = true;
                shootTimer.reset();
                break;
            case 2:
                // Shooting done after Path1, start Path2 with intake ON
                shooter.blockShooter();
                shooterRunning = false;
                drive.followPathChain(paths.Path2, true);
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                break;
            case 3:
                // Path2 done, wait for balls to fall
                IntakeTimer.reset();
                break;
            case 4:
                // Start Path3 back to shooting position
                shooter.setTurretAngle(-90);
                shooterRunning = true;
                drive.followPathChain(paths.Path3, true);
                break;
            case 5:
                shooting = true;
                shootTimer.reset();
                break;
            case 6:
                // Shooting done after Path1, start Path2 with intake ON
                shooter.blockShooter();
                shooterRunning = false;
                drive.followPathChain(paths.Path4, true);
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                break;
            case 7:
                // Path2 done, wait for balls to fall
                IntakeTimer.reset();
                break;
            case 8:
                // Start Path3 back to shooting position
                shooter.setTurretAngle(-90);
                shooterRunning = true;
                drive.followPathChain(paths.Path5, true);
                break;
            case 9:
                // End of Path3: wait for shooter ready, then shoot
                shooter.stopIntakeSystem();
                shooterRunning = true;
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
                if (shooting && shooter.isShooterReady(targetShooterVelocity, limelight.isAlignedForShooting())) {
                    shooter.unblockShooter();
                    shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                    shootTimer.reset();
                    shooting = false;
                }
                if (!shooting && shootTimer.seconds() >= SHOOT_TIME) {
                    shooter.blockShooter();
                    shooter.stopIntakeSystem();
                    setPathState(2); // Shooting done, go to gate
                }
                break;
            case 2:
                // Following Path2 to gate with intake
                if (!drive.isBusy()) {
                    setPathState(3); // Wait for balls to fall
                }
                break;
            case 3:
                // Wait for balls to fall into intake
                if (!drive.isBusy() && (shooter.intakeFull() || IntakeTimer.seconds() > 1.5)) {
                    setPathState(4); // Go back to shooting position
                }
                break;
            case 4:
                // Following Path3 back to shooting position
                if (!drive.isBusy()) {
                    setPathState(5); // Wait for shooter ready, then shoot
                }
                break;
            case 5:
                if (shooting && shooter.isShooterReady(targetShooterVelocity, limelight.isAlignedForShooting())) {
                    shooter.unblockShooter();
                    shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                    shootTimer.reset();
                    shooting = false;
                }
                if (!shooting && shootTimer.seconds() >= SHOOT_TIME) {
                    shooter.blockShooter();
                    shooter.stopIntakeSystem();
                    setPathState(6); // Loop back to gate
                }
                break;
            case 6:
                if(!drive.isBusy()){
                    setPathState(7);
                }
                break;
            case 7:
                if(!drive.isBusy() && (shooter.intakeFull() || IntakeTimer.seconds() > 1.75)){
                    setPathState(8);
                }
                break;
            case 8:
                if(!drive.isBusy()) {
                    setPathState(9); // Shooting done, go to gate
                }
                break;
            case 9:
                // Wait for shooter ready, then open blocker and flush balls
                if (shooting && shooter.isShooterReady(targetShooterVelocity, limelight.isAlignedForShooting())) {
                    shooter.unblockShooter();
                    shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                    shootTimer.reset();
                    shooting = false;
                }
                if (!shooting && shootTimer.seconds() >= SHOOT_TIME) {
                    shooter.blockShooter();
                    shooter.stopIntakeSystem();
                    setPathState(3); // Loop back to gate
                }
                break;
        }
    }
}
