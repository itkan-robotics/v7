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
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Autonomous(name = "RED AUTO", group = "Autonomous")
@Configurable // Panels
public class REDAUTO extends LinearOpMode {
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
    private static final double SHOOT_TIME = 0.5; // seconds for flushing balls

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
    private final Pose2D startPose = new Pose2D(DistanceUnit.INCH, 112, 135, AngleUnit.RADIANS, Math.toRadians(-90));

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

            panelsTelemetry.debug("SHOOTER VEL >>>>   ", shooter.getShooterVelocity());
            panelsTelemetry.debug("SHOOTER TARGET >>>> ", targetShooterVelocity);

            panelsTelemetry.update(telemetry);
        }
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

        public Paths(Follower follower) {
            StartToShot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(112.000, 135.000), new Pose(95.000, 88.000))
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
                            new BezierLine(new Pose(119.000, 60), new Pose(87.000, 72.000))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
            // Intake run path - starts where Path3 ends
            Shoot2ToLever = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(87.000, 72.000), new Pose(106.250, 63.5)) //64
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-13), Math.toRadians(20))

                    .addPath(
                            new BezierLine(new Pose(106.250, 63.5), new Pose(134,63.5))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(20))
                    .build();

            // Return from lever to shooting position
            LeverToShoot3 = follower

                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(133.5, 64), new Pose(87.000, 73.000))
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
                            new BezierLine(new Pose(87.000, 73.000), new Pose(127.000, 84))
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
                            new BezierLine(new Pose(125, 39.5), new Pose(87.000, 75.000))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            End = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(87.000, 75.000), new Pose(116, 70.603))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
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
                shooter.setTurretAngle(-18);
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
                shooter.setTurretAngle(-79);
                shooterRunning = true;
                drive.followPathChain(paths.Tape2ToSHoot2, true);
                pathTimer.reset();
                break;
            case 5:
                shooter.setIndexerMiddle();

                // Shoot at position 2
                shooting = true;
                shootTimer.reset();
                break;

            // === TO LEVER (first cycle) ===
            case 6:
                // Shoot2ToLever: Close blocker, turn off shooter, run intake
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                drive.followPathChain(paths.Shoot2ToLever, true);
                pathTimer.reset();
                break;
            case 7:
                // Wait at lever
                IntakeTimer.reset();
                break;

            // === THIRD SHOT ===
            case 8:
                // LeverToShoot3: Set turret to -75, stop intake, turn on shooter
                shooter.setTurretAngle(-82);
                shooterRunning = true;
                drive.followPathChain(paths.LeverToShoot3, true);
                pathTimer.reset();
                break;
            case 9:
                shooter.setIndexerMiddle();

                // Shoot at position 3
                shooting = true;
                shootTimer.reset();
                break;

            // === TO LEVER (second cycle) ===
            case 10:
                // Shoot2ToLever: Close blocker, turn off shooter, run intake
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                drive.followPathChain(paths.Shoot2ToLever, true);
                pathTimer.reset();
                break;
            case 11:
                // Wait at lever
                IntakeTimer.reset();
                break;

            // === FOURTH SHOT ===
            case 12:
                // LeverToShoot3: Set turret to -75, stop intake, turn on shooter
                shooter.setTurretAngle(-82);

                shooterRunning = true;
                drive.followPathChain(paths.LeverToShoot3, true);
                pathTimer.reset();
                break;
            case 13:
                shooter.setIndexerMiddle();

                // Shoot at position 4
                shooting = true;
                shootTimer.reset();
                break;

            // === TO LEVER (third cycle) ===
            case 14:
                // Shoot2ToLever: Close blocker, turn off shooter, run intake
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                drive.followPathChain(paths.Shoot2ToLever, true);
                pathTimer.reset();
                break;
            case 15:
                // Wait at lever
                IntakeTimer.reset();
                break;

            // === FIFTH SHOT (from lever) ===
            case 16:
                // LeverToShoot: Set turret, stop intake, turn on shooter
                shooter.setTurretAngle(-82);
                shooterRunning = true;
                drive.followPathChain(paths.LeverToShoot3, true);
                pathTimer.reset();
                break;
            case 17:
                shooter.setIndexerMiddle();

                // Shoot at position 5 (from lever)
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

            // === SIXTH SHOT (from tape 1) ===
            case 20:
                // tape1ToShoot4: Go to shooting position
                shooter.setTurretAngle(-85);
              //  shooter.stopIntakeSystem();
                shooterRunning = true;
                drive.followPathChain(paths.tape1ToShoot4, true);
                pathTimer.reset();
                break;
            case 21:
                shooter.setIndexerMiddle();

                // Shoot at position 6
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
                shooter.setTurretAngle(-65);

                // CornertoShoot5: Return to final shooting position
                shooter.stopIntakeSystem();
                shooterRunning = true;
                drive.followPathChain(paths.CornertoShoot5, true);
                pathTimer.reset();
                break;
            case 25:
                shooter.setIndexerMiddle();
                // Final shoot
                shooting = true;
                shootTimer.reset();
                break;

            case 26:
                drive.followPathChain(paths.End, true);
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
                if (!drive.isBusy()) {
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

            // === TO LEVER (first cycle) ===
            case 6:
                // Following Shoot2ToLever
                if (!drive.isBusy()) {
                    setPathState(7);
                }
                break;
            case 7:
                // Wait at lever
                if (!drive.isBusy() && (shooter.issintakeFull() || IntakeTimer.seconds() > 1.5)) {
                    setPathState(8);
                }
                break;

            // === THIRD SHOT ===
            case 8:
                // Following LeverToShoot3
                if(pathTimer.seconds() > 0.5) {
                    shooter.stopIntakeSystem();
                }
                if (!drive.isBusy()) {
                    setPathState(9);
                }
                break;
            case 9:
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
                        setPathState(10);
                    }
                }
                break;

            // === TO LEVER (second cycle) ===
            case 10:
                // Following Shoot2ToLever
                if (!drive.isBusy()) {
                    setPathState(11);
                }
                break;
            case 11:
                // Wait at lever
                if (!drive.isBusy() && (shooter.issintakeFull() || IntakeTimer.seconds() > 1.5)) {
                    setPathState(12);
                }
                break;

            // === FOURTH SHOT ===
            case 12:

                if(pathTimer.seconds() > 0.5) {
                    shooter.stopIntakeSystem();
                }
                // Following LeverToShoot3
                if (!drive.isBusy()) {
                    setPathState(13);
                }
                break;
            case 13:
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
                        setPathState(14);
                    }
                }
                break;

            // === TO LEVER (third cycle) ===
            case 14:
                // Following Shoot2ToLever
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

            // === FIFTH SHOT (from lever) ===
            case 16:

                if(pathTimer.seconds() > 0.5) {
                    shooter.stopIntakeSystem();
                }
                // Following LeverToShoot3
                if (!drive.isBusy()) {
                    setPathState(17);
                }
                break;
            case 17:
                // Shoot at position 5 (from lever)
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
                if (!drive.isBusy()) {
                    setPathState(20);
                }
                break;

            // === SIXTH SHOT (from tape 1) ===
            case 20:
                // Following tape1ToShoot4
                if (!drive.isBusy()) {
                    setPathState(21);
                }
                break;
            case 21:
                // Shoot at position 6
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
                if (!drive.isBusy()) {
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
                        setPathState(26);
                        // Autonomous complete - stay in state 25
                    }
                }
                break;


        }
    }
}
