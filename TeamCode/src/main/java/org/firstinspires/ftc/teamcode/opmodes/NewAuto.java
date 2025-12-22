package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

@Autonomous(name = "NewAuto", group = "Autonomous")
@Configurable // Panels
public class NewAuto extends LinearOpMode {
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
            //limelight.update(); // Update Limelight
            autonomousPathUpdate(); // Update autonomous state machine

            // Update shooter target velocity from limelight
//            targetShooterVelocity = updateTargetShooterVelocity();
            shooter.updateShooter(shooterRunning, targetShooterVelocity);

            //Pose2D currentPose = drive.getCurrentPose();
//            panelsTelemetry.debug("X", currentPose.getX(DistanceUnit.INCH));
//            panelsTelemetry.debug("Y", currentPose.getY(DistanceUnit.INCH));
//            panelsTelemetry.debug("Heading", currentPose.getHeading(AngleUnit.RADIANS));
            panelsTelemetry.debug("SHOOTER VEL >>>>   ", shooter.getShooterVelocity());
            panelsTelemetry.debug("SHOOTER TARGET >>>> ", targetShooterVelocity);
//            panelsTelemetry.debug("intake full", shooter.issintakeFull());
//            panelsTelemetry.debug("Power Consumption", shooter.getPowerConsumption());
            panelsTelemetry.update(telemetry);
        }
    }


    public static class Paths {

        public PathChain StartToShot;
        public PathChain Shoot1ToTape2;
        public PathChain Tape2ToSHoot2;
        public PathChain Shoot2ToGate1;          // Intake run: (83.457, 72.093) → (117, 72)
        public PathChain Gate1toGateLever;   // To shooting: (117, 72) → (133.568, 62.250)
        public PathChain LeverToShoot3;          // Return from shooting: (133.568, 62.250) → (83.457, 72.093)
        public PathChain Shoot3ToTape1;
        public PathChain tape1ToShoot4;
        public PathChain Shoot4toCorner;
        public PathChain CornertoShoot5;

        public Paths(Follower follower) {
            StartToShot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(128.724, 113.263), new Pose(92.000, 88.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                    .build();

            Shoot1ToTape2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(92.000, 88.000),
                                    new Pose(92.000, 58.000),
                                    new Pose(123.000, 60.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Tape2ToSHoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(123.000, 60.000),
                                    new Pose(92.000, 58.000),
                                    new Pose(87.000, 64.000),
                                    new Pose(83.457, 72.093)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            // Intake run path - starts where Path3 ends
            Shoot2ToGate1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(83.457, 72.093), new Pose(90.000, 72.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                    .addPath(
                            new BezierLine(new Pose(90.000, 72.000), new Pose(117.000, 72.000))
                    )
                    .build();

            // To shooting position - starts where Path4 ends
            Gate1toGateLever = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(117.000, 72.000),
                                    new Pose(125.930, 60.000),
                                    new Pose(133.568, 62.250)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            // Return from shooting to intake start - starts where Path4ToShoot ends
            LeverToShoot3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(133.568, 62.250), new Pose(125.371, 63.524))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-12))
                    .addPath(
                            new BezierLine(new Pose(125.371, 63.524), new Pose(83.457, 72.093))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            // Intake along upper path - starts where Path5 ends
            Shoot3ToTape1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(83.457, 72.093),
                                    new Pose(83.000, 91.000),
                                    new Pose(121.646, 85.692)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                    .build();

            // To shooting position from Path6
            tape1ToShoot4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(121.646, 85.692),
                                    new Pose(94.000, 86.000),
                                    new Pose(94.000, 86.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                    .build();

            // Intake to far corner - starts where Path7 ends
            Shoot4toCorner = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(94.000, 86.000),
                                    new Pose(94.000, 86.000),
                                    new Pose(86.000, 94.000),
                                    new Pose(133.382, 23.472)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-45))
                    .build();

            // Return from far corner to shooting - starts where Path10 ends
            CornertoShoot5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(133.382, 23.472), new Pose(83.457, 71.907))
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

    private boolean limelightTurretAutoAlign() {
        if (!limelight.hasTarget()) {
            return false;
        }

        int tagId = limelight.getAprilTagId();
        if (tagId != 24) {
            return false;
        }

        double tx = limelight.getTx();
        double targetOffset = shooter.calculateTargetOffset(limelight, tagId);
        double error = tx - targetOffset;

        if (Math.abs(error) > shooter.LIMELIGHT_TOLERANCE) {
            double currentTurretAngle = shooter.getTurretAngle();
            double turretAdjustment = -error * shooter.LIMELIGHT_KP;
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
    public void setPathState(int state) {
        pathState = state;
        switch (pathState) {
            // === FIRST SHOT ===
            case 0:
                // StartToShot: Go to first shooting position
                shooter.setTurretAngle(-34);
                shooter.setIndexerMiddle();
                shooterRunning = true;
                drive.followPathChain(paths.StartToShot, true);
                pathTimer.reset();
                break;
            case 1:
                // Shoot at position 1
                shooting = true;
                shootTimer.reset();
                break;

            // === TO TAPE 2 ===
            case 2:
                // Shoot1ToTape2: Go to tape 2 with intake
                shooter.blockShooter();
                shooterRunning = false;
                drive.followPathChain(paths.Shoot1ToTape2, true);
                pathTimer.reset();
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                break;
            case 3:
                // Wait for intake at tape 2
                IntakeTimer.reset();
                break;

            // === SECOND SHOT ===
            case 4:
                // Tape2ToShoot2: Return to shooting position 2
                shooter.setTurretAngle(-52);
                shooterRunning = true;
                drive.followPathChain(paths.Tape2ToSHoot2, true);
                pathTimer.reset();
                break;
            case 5:
                // Shoot at position 2
                shooting = true;
                shootTimer.reset();
                break;

            // === TO GATE 1 ===
            case 6:
                // Shoot2ToGate1: Close blocker, turn off shooter, run intake
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                drive.followPathChain(paths.Shoot2ToGate1, true);
                pathTimer.reset();
                break;
            case 7:
                // Wait for intake at gate 1
                IntakeTimer.reset();
                break;

            // === TO GATE LEVER ===
            case 8:
                // Gate1toGateLever: Continue with intake on
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                drive.followPathChain(paths.Gate1toGateLever, true);
                pathTimer.reset();
                break;
            case 9:
                // Wait at lever
                IntakeTimer.reset();
                break;

            // === THIRD SHOT ===
            case 10:
                // LeverToShoot3: Set turret to -52, stop intake, turn on shooter
                shooter.setTurretAngle(-52);
                shooter.stopIntakeSystem();
                shooterRunning = true;
                drive.followPathChain(paths.LeverToShoot3, true);
                pathTimer.reset();
                break;
            case 11:
                // Shoot at position 3
                shooting = true;
                shootTimer.reset();
                break;

            // === REPEAT: TO GATE 1 (second cycle) ===
            case 12:
                // Shoot2ToGate1: Close blocker, turn off shooter, run intake
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                drive.followPathChain(paths.Shoot2ToGate1, true);
                pathTimer.reset();
                break;
            case 13:
                // Wait for intake at gate 1
                IntakeTimer.reset();
                break;

            // === TO GATE LEVER (second cycle) ===
            case 14:
                // Gate1toGateLever: Continue with intake on
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                drive.followPathChain(paths.Gate1toGateLever, true);
                pathTimer.reset();
                break;
            case 15:
                // Wait at lever
                IntakeTimer.reset();
                break;

            // === FOURTH SHOT ===
            case 16:
                // LeverToShoot3: Set turret to -52, stop intake, turn on shooter
                shooter.setTurretAngle(-52);
                shooter.stopIntakeSystem();
                shooterRunning = true;
                drive.followPathChain(paths.LeverToShoot3, true);
                pathTimer.reset();
                break;
            case 17:
                // Shoot at position 4
                shooting = true;
                shootTimer.reset();
                break;

            // === TO TAPE 1 ===
            case 18:
                // Shoot3ToTape1: Go to tape 1 with intake
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                drive.followPathChain(paths.Shoot3ToTape1, true);
                pathTimer.reset();
                break;
            case 19:
                // Wait for intake at tape 1
                IntakeTimer.reset();
                break;

            // === FIFTH SHOT ===
            case 20:
                // tape1ToShoot4: Go to shooting position 4
                shooter.setTurretAngle(-45);
                shooter.stopIntakeSystem();
                shooterRunning = true;
                drive.followPathChain(paths.tape1ToShoot4, true);
                pathTimer.reset();
                break;
            case 21:
                // Shoot at position 5
                shooting = true;
                shootTimer.reset();
                break;

            // === TO CORNER ===
            case 22:
                // Shoot4toCorner: Go to corner with intake
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                drive.followPathChain(paths.Shoot4toCorner, true);
                pathTimer.reset();
                break;
            case 23:
                // Wait for intake at corner
                IntakeTimer.reset();
                break;

            // === FINAL SHOT ===
            case 24:
                // CornertoShoot5: Return to final shooting position
                shooter.stopIntakeSystem();
                shooterRunning = true;
                drive.followPathChain(paths.CornertoShoot5, true);
                pathTimer.reset();
                break;
            case 25:
                // Final shoot
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
            // === FIRST SHOT ===
            case 0:
                // Following StartToShot to first shooting position
                if (!drive.isBusy()) {
                    setPathState(1);
                }
                break;
            case 1:
                // Shoot at position 1
                if (!drive.isBusy()) {
                    limelight.update();
                    limelightTurretAutoAlign();
                    targetShooterVelocity = updateTargetShooterVelocity();
                    shooter.updateShooter(shooterRunning, targetShooterVelocity);
                    if (shooting && shooter.isShooterReady(targetShooterVelocity, limelight.isAlignedForShooting())) {
                        shooter.unblockShooter();
                        shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                        shootTimer.reset();
                        shooting = false;
                    }
                    if (!shooting && shootTimer.seconds() >= SHOOT_TIME) {
                        shooter.blockShooter();
                        shooter.stopIntakeSystem();
                        setPathState(2);
                    }
                }
                break;

            // === TO TAPE 2 ===
            case 2:
                // Following Shoot1ToTape2
                if (!drive.isBusy()) {
                    setPathState(3);
                }
                break;
            case 3:
                // Wait for intake at tape 2
                if (!drive.isBusy() && (shooter.intakeFull() || IntakeTimer.seconds() > 2)) {
                    setPathState(4);
                }
                break;

            // === SECOND SHOT ===
            case 4:
                // Following Tape2ToShoot2
                if (!drive.isBusy()) {
                    setPathState(5);
                }
                break;
            case 5:
                // Shoot at position 2
                if (!drive.isBusy()) {
                    limelight.update();
                    limelightTurretAutoAlign();
                    targetShooterVelocity = updateTargetShooterVelocity();
                    shooter.updateShooter(shooterRunning, targetShooterVelocity);
                    if (shooting && shooter.isShooterReady(targetShooterVelocity, limelight.isAlignedForShooting())) {
                        shooter.unblockShooter();
                        shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                        shootTimer.reset();
                        shooting = false;
                    }
                    if (!shooting && shootTimer.seconds() >= SHOOT_TIME) {
                        shooter.blockShooter();
                        shooter.stopIntakeSystem();
                        setPathState(6);
                    }
                }
                break;

            // === TO GATE 1 ===
            case 6:
                // Following Shoot2ToGate1
                if (!drive.isBusy()) {
                    setPathState(7);
                }
                break;
            case 7:
                // Wait for intake at gate 1
                if (!drive.isBusy()) {
                    setPathState(8);
                }
                break;

            // === TO GATE LEVER ===
            case 8:
                // Following Gate1toGateLever
                if (!drive.isBusy()) {
                    setPathState(9);
                }
                break;
            case 9:
                // Wait at lever
                if (!drive.isBusy() && (shooter.issintakeFull() || IntakeTimer.seconds() > 1.5)) {
                    setPathState(10);
                }
                break;

            // === THIRD SHOT ===
            case 10:
                // Following LeverToShoot3
                if (!drive.isBusy()) {
                    setPathState(11);
                }
                break;
            case 11:
                // Shoot at position 3
                if (!drive.isBusy()) {
                    limelight.update();
                    limelightTurretAutoAlign();
                    targetShooterVelocity = updateTargetShooterVelocity();
                    shooter.updateShooter(shooterRunning, targetShooterVelocity);
                    if (shooting && shooter.isShooterReady(targetShooterVelocity, limelight.isAlignedForShooting())) {
                        shooter.unblockShooter();
                        shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                        shootTimer.reset();
                        shooting = false;
                    }
                    if (!shooting && shootTimer.seconds() >= SHOOT_TIME) {
                        shooter.blockShooter();
                        shooter.stopIntakeSystem();
                        setPathState(12);
                    }
                }
                break;

            // === REPEAT: TO GATE 1 (second cycle) ===
            case 12:
                // Following Shoot2ToGate1
                if (!drive.isBusy()) {
                    setPathState(13);
                }
                break;
            case 13:
                // Wait for intake at gate 1
                if (!drive.isBusy() && (shooter.issintakeFull() || IntakeTimer.seconds() > 2)) {
                    setPathState(14);
                }
                break;

            // === TO GATE LEVER (second cycle) ===
            case 14:
                // Following Gate1toGateLever
                if (!drive.isBusy()) {
                    setPathState(15);
                }
                break;
            case 15:
                // Wait at lever
                if (!drive.isBusy() && (shooter.issintakeFull() || IntakeTimer.seconds() > 1.5)) {
                    setPathState(16);
                }
                break;

            // === FOURTH SHOT ===
            case 16:
                // Following LeverToShoot3
                if (!drive.isBusy()) {
                    setPathState(17);
                }
                break;
            case 17:
                // Shoot at position 4
                if (!drive.isBusy()) {
                    limelight.update();
                    limelightTurretAutoAlign();
                    targetShooterVelocity = updateTargetShooterVelocity();
                    shooter.updateShooter(shooterRunning, targetShooterVelocity);
                    if (shooting && shooter.isShooterReady(targetShooterVelocity, limelight.isAlignedForShooting())) {
                        shooter.unblockShooter();
                        shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                        shootTimer.reset();
                        shooting = false;
                    }
                    if (!shooting && shootTimer.seconds() >= SHOOT_TIME) {
                        shooter.blockShooter();
                        shooter.stopIntakeSystem();
                        setPathState(18);
                    }
                }
                break;

            // === TO TAPE 1 ===
            case 18:
                // Following Shoot3ToTape1
                if (!drive.isBusy()) {
                    setPathState(19);
                }
                break;
            case 19:
                // Wait for intake at tape 1
                if (!drive.isBusy() && (shooter.issintakeFull() || IntakeTimer.seconds() > 1.75)) {
                    setPathState(20);
                }
                break;

            // === FIFTH SHOT ===
            case 20:
                // Following tape1ToShoot4
                if (!drive.isBusy()) {
                    setPathState(21);
                }
                break;
            case 21:
                // Shoot at position 5
                if (!drive.isBusy()) {
                    limelight.update();
                    limelightTurretAutoAlign();
                    targetShooterVelocity = updateTargetShooterVelocity();
                    shooter.updateShooter(shooterRunning, targetShooterVelocity);
                    if (shooting && shooter.isShooterReady(targetShooterVelocity, limelight.isAlignedForShooting())) {
                        shooter.unblockShooter();
                        shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                        shootTimer.reset();
                        shooting = false;
                    }
                    if (!shooting && shootTimer.seconds() >= SHOOT_TIME) {
                        shooter.blockShooter();
                        shooter.stopIntakeSystem();
                        setPathState(22);
                    }
                }
                break;

            // === TO CORNER ===
            case 22:
                // Following Shoot4toCorner
                if (!drive.isBusy()) {
                    setPathState(23);
                }
                break;
            case 23:
                // Wait for intake at corner
                if (!drive.isBusy() && (shooter.issintakeFull() || IntakeTimer.seconds() > 1.75)) {
                    setPathState(24);
                }
                break;

            // === FINAL SHOT ===
            case 24:
                // Following CornertoShoot5
                if (!drive.isBusy()) {
                    setPathState(25);
                }
                break;
            case 25:
                // Final shooting sequence
                if (!drive.isBusy()) {
                    limelight.update();
                    limelightTurretAutoAlign();
                    targetShooterVelocity = updateTargetShooterVelocity();
                    shooter.updateShooter(shooterRunning, targetShooterVelocity);
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
                        // Autonomous complete - stay in state 25
                    }
                }
                break;
        }
    }
}
