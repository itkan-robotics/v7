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

@Autonomous(name = "BLUE AUTO", group = "Autonomous")
@Configurable // Panels
public class BLUEAUTO extends LinearOpMode {
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
    private static final double SHOOT_TIME = 1; // seconds for flushing balls

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
    private final Pose2D startPose = new Pose2D(DistanceUnit.INCH, 15.27554980595085, 113.26261319534282, AngleUnit.RADIANS, Math.toRadians(90));

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

            //Pose2D currentPose = drive.getCurrentPose();
//            panelsTelemetry.debug("X", currentPose.getX(DistanceUnit.INCH));
//            panelsTelemetry.debug("Y", currentPose.getY(DistanceUnit.INCH));
//            panelsTelemetry.debug("Heading", currentPose.getHeading(AngleUnit.RADIANS));
            // Auto-align turret while moving to shooting positions
            if (pathState == 0 || pathState == 1 ||
                    pathState == 4 || pathState == 5 ||
                    pathState == 8 || pathState == 9 ||
                    pathState == 12 || pathState == 13 ||
                    pathState == 16 || pathState == 17 ||
                    pathState == 20 || pathState == 21) {
                shooter.redlimelightTurretAutoAlign(limelight);
            }

            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
//            Pose2D currentPose = drive.getCurrentPose();
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
                            new BezierLine(new Pose(15.276, 113.263), new Pose(52.000, 88.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            Shoot1ToTape2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(52.000, 88.000),
                                    new Pose(52.000, 59.000),
                                    new Pose(21.000, 60.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Tape2ToSHoot2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(21.000, 60.000),
                                    new Pose(52.000, 58.000),
                                    new Pose(57.000, 64.000),
                                    new Pose(60.543, 72.093)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(191))
                    .build();

            // Intake run path - starts where Path3 ends
            Shoot2ToLever = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.543, 72.000), new Pose(52.160, 70.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(191))

                    .addPath(
                            new BezierLine(new Pose(52.160, 70.000), new Pose(44, 66.00))
                    )
                    .setTangentHeadingInterpolation()

                    .addPath(
                            new BezierCurve(
                                    new Pose(44.000, 66.00),
                                    new Pose(27.000, 56.000),
                                    new Pose(9.25, 62)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(191), Math.toRadians(150))
                    .build();

            // Return from lever to shooting position
            LeverToShoot3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(11, 62.5), new Pose(32.414, 65.014))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(191))
                    .addPath(
                            new BezierLine(new Pose(18.629, 63.524), new Pose(60.543, 72.093))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            // Intake along upper path - starts where Path5 ends
            Shoot3ToTape1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.543, 72.093),
                                    new Pose(61.000, 91.000),
                                    new Pose(22.354, 85.692)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(191), Math.toRadians(180))
                    .build();

            // To shooting position from Path6
            tape1ToShoot4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(22.354, 85.692),
                                    new Pose(50.000, 86.000),
                                    new Pose(50.000, 86.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(191))
                    .build();

            // Intake to far corner - starts where Path7 ends
            Shoot4toCorner = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(50.000, 86.000), new Pose(53.464, 63.151))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(191), Math.toRadians(225))

                    .addPath(
                            new BezierLine(new Pose(53.464, 63.151), new Pose(10, 21))
                    )
                    .setTangentHeadingInterpolation()
                    .build();


            // Return from far corner to shooting - starts where Path10 ends
            CornertoShoot5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(10, 21), new Pose(61, 74.000))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            End = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(61.000, 74.000), new Pose(26, 70.603))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(90))
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
        if (tagId != 20) {  // Blue alliance uses tag 20
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
                shooter.setTurretAngle(34);
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
                shooter.setTurretAngle(83);
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
                // LeverToShoot3: Set turret to 83, stop intake, turn on shooter
                shooter.setTurretAngle(83);
                shooter.stopIntakeSystem();
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
                // LeverToShoot3: Set turret to 83, stop intake, turn on shooter
                shooter.setTurretAngle(83);
                shooter.stopIntakeSystem();
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

            // === TO TAPE 1 ===
            case 14:
                // Shoot3ToTape1: Go to tape 1 with intake
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                drive.followPathChain(paths.Shoot3ToTape1, true);
                pathTimer.reset();
                break;
            case 15:
                // Wait for intake at tape 1
                IntakeTimer.reset();
                break;

            // === FIFTH SHOT ===
            case 16:
                // tape1ToShoot4: Go to shooting position 4
                shooter.setTurretAngle(81);
                shooter.stopIntakeSystem();
                shooterRunning = true;
                drive.followPathChain(paths.tape1ToShoot4, true);
                pathTimer.reset();
                break;
            case 17:
                shooter.setIndexerMiddle();

                // Shoot at position 5
                shooting = true;
                shootTimer.reset();
                break;

            // === TO CORNER ===
            case 18:
                // Shoot4toCorner: Go to corner with intake
                shooter.blockShooter();
                shooterRunning = false;
                shooter.runIntakeSystem(Shooter.INTAKE_POWER);
                drive.followPathChain(paths.Shoot4toCorner, true);
                pathTimer.reset();
                break;
            case 19:
                // Wait for intake at corner
                IntakeTimer.reset();
                break;

            // === FINAL SHOT ===
            case 20:
                shooter.setTurretAngle(60);

                // CornertoShoot5: Return to final shooting position
                shooter.stopIntakeSystem();
                shooterRunning = true;
                drive.followPathChain(paths.CornertoShoot5, true);
                pathTimer.reset();
                break;
            case 21:
                shooter.setIndexerMiddle();
                // Final shoot
                shooting = true;
                shootTimer.reset();
                break;

            case 22:
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
                if (!drive.isBusy() && (shooter.issintakeFull() || IntakeTimer.seconds() > 1.75)) {
                    setPathState(8);
                }
                break;

            // === THIRD SHOT ===
            case 8:
                // Following LeverToShoot3
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
                if (!drive.isBusy() && (shooter.issintakeFull() || IntakeTimer.seconds() > 1.75)) {
                    setPathState(12);
                }
                break;

            // === FOURTH SHOT ===
            case 12:
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

            // === TO TAPE 1 ===
            case 14:
                // Following Shoot3ToTape1
                if (!drive.isBusy()) {
                    setPathState(15);
                }
                break;
            case 15:
                // Wait for intake at tape 1
                if (!drive.isBusy()) {
                    setPathState(16);
                }
                break;

            // === FIFTH SHOT ===
            case 16:
                // Following tape1ToShoot4
                if (!drive.isBusy()) {
                    setPathState(17);
                }
                break;
            case 17:
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
                        setPathState(18);
                    }
                }
                break;

            // === TO CORNER ===
            case 18:
                // Following Shoot4toCorner
                if (!drive.isBusy()) {
                    setPathState(19);
                }
                break;
            case 19:
                // Wait for intake at corner
                if (!drive.isBusy() && (shooter.issintakeFull() || IntakeTimer.seconds() > 0.5)) {
                    setPathState(20);
                }
                break;

            // === FINAL SHOT ===
            case 20:
                // Following CornertoShoot5
                if (!drive.isBusy()) {
                    setPathState(21);
                }
                break;
            case 21:
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
                        setPathState(22);
                        // Autonomous complete - stay in state 21
                    }
                }
                break;


        }
    }
}
