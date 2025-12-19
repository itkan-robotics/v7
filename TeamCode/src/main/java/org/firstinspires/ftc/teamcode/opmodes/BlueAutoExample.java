package org.firstinspires.ftc.teamcode.opmodes;

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

@Autonomous(name = "Blue Alliance Autonomous", group = "Autonomous")
public class BlueAutoExample extends LinearOpMode {
    private Drive drive;
    private Limelight limelight;
    private Shooter shooter;

    private int pathState; // Current autonomous path state (state machine)
    private Drive.BlueAutonomousPaths paths; // Paths for blue alliance (mirrored)

    // Timer for shooting phases
    private ElapsedTime shootTimer = new ElapsedTime();
    private static final double SHOOT_TIME = 0.5; // seconds

    // Timer for gate wait
    private ElapsedTime gateWaitTimer = new ElapsedTime();
    private static final double GATE_WAIT_TIME = 2.5; // seconds

    // Flag for shooter control - must be updated continuously for bang-bang
    private boolean shooterRunning = false;

    // Target AprilTag ID for shooting (20 = Blue alliance backboard)
    private static final int TARGET_TAG_ID = 20;

    // Starting pose - mirrored from red (129.0, 107.0) -> (15.0, 107.0)
    // X coordinate: 144 - 129 = 15
    private final Pose2D startPose = new Pose2D(DistanceUnit.INCH, 15.0, 107.0, AngleUnit.RADIANS, Math.toRadians(-90));

    // Target shooter velocity (will be updated from limelight)
    private double targetShooterVelocity = Shooter.DEFAULT_TARGET_SHOOTER_VELOCITY;

    @Override
    public void runOpMode() throws InterruptedException {
        // === INIT ===
        drive = new Drive(hardwareMap);
        limelight = new Limelight(hardwareMap);
        shooter = new Shooter(hardwareMap);

        drive.setStartingPose(startPose);
        paths = new Drive.BlueAutonomousPaths(drive.getFollower());
        pathState = 0;

        limelight.start();
        limelight.switchPipeline(0);
        shooter.blockShooter(); // Start with blocker closed

        telemetry.addLine("INITIALIZED SUCCESSFULLY");
        telemetry.addData("Target Tag ID", TARGET_TAG_ID + " (Blue)");
        telemetry.addLine("Will shoot 3 balls at each shooting position");
        telemetry.update();

        // === INIT LOOP ===
        while (!isStarted() && !isStopRequested()) {
            limelight.update();
            shooter.blockShooter();
            sleep(50);
        }

        // === START ===
        setPathState(0);

        // === MAIN LOOP ===
        // Optimized for faster loop times to improve bang-bang controller accuracy
        telemetry.setMsTransmissionInterval(50); // Reduce telemetry frequency for better loop times

        while (opModeIsActive()) {
            // Critical updates first (must be called every loop for accuracy)
            drive.update(); // Update Pedro Pathing
            limelight.update(); // Update Limelight

            // Update shooter target velocity (minimal check)
            if (limelight.hasTarget()) {
                targetShooterVelocity = limelight.getTargetShooterVelocity();
            } else {
                targetShooterVelocity = Shooter.DEFAULT_TARGET_SHOOTER_VELOCITY;
            }

            // Update shooter bang-bang controller (critical - must be called every loop)
            shooter.updateShooterWithHold(shooterRunning, targetShooterVelocity);

            // Update autonomous state machine
            autonomousPathUpdate();

            // Minimal telemetry (only essential data, reduced frequency)
            telemetry.addData("State", pathState);
            telemetry.addData("Shooter", "%.0f/%.0f", shooter.getShooterVelocity(), targetShooterVelocity);
            telemetry.addData("Tag", limelight.hasTarget() && limelight.hasAprilTag(TARGET_TAG_ID));

            telemetry.update();
        }
    }

    /**
     * Shoot all three balls by running shooter and blocker logic for 1.5 seconds
     * Alignment happens in parallel with firing
     */
    private void shootAllThreeBalls() throws InterruptedException {
        // Run shooter and blocker logic for 1.5 seconds while aligning in parallel
        ElapsedTime shootTimer = new ElapsedTime();
        shootTimer.reset();

        while (opModeIsActive() && shootTimer.seconds() < 1.5) {
            limelight.update();
            drive.update();

            // Update target velocity from limelight
            if (limelight.hasTarget()) {
                targetShooterVelocity = limelight.getTargetShooterVelocity();
            }

            // Update shooter bang-bang controller
            shooter.updateShooterWithHold(true, targetShooterVelocity);
            if(limelight.isAligned()) {
                // Unblock blocker to feed balls
                shooter.unblockShooter();
                shooter.setIntakeState(Shooter.IntakeState.INTAKING);
            }

            // Align to target tag (20 = Blue) in parallel (non-blocking)
            if (limelight.hasTarget() && limelight.hasAprilTag(TARGET_TAG_ID)) {
                double targetTx = 0.0; // Target tx for alignment (center on target)

                // Calculate alignment power using limelight's aim assist
                double drivePower = limelight.calculateAimAssistPower(targetTx, true);

                // Turn in place to align (only rotation, doesn't interfere with shooting)
                drive.driveTeleop(0, 0, drivePower, false);
            } else {
                // No target tag visible, stop turning
                drive.driveTeleop(0, 0, 0, false);
            }

            sleep(20);
        }

        // Stop shooting and block
        shooter.blockShooter();
        shooter.setIntakeState(Shooter.IntakeState.HOLD);
        drive.driveTeleop(0, 0, 0, false); // Stop turning
    }


    /**
     * Sets the path state and starts following the corresponding path
     * @param state the path state to transition to
     */
    public void setPathState(int state) {
        pathState = state;
        switch (pathState) {
            case 0:
                // Start to first shooting position
                shooterRunning = true;
                drive.followPathChain(paths.startToShooting1, true);
                break;
            case 1:
                // End of startToShooting1: ready to shoot (shooting handled in autonomousPathUpdate)
                shooterRunning = true;
                break;
            case 2:
                // Shooting done, start intake BEFORE path to intake area
                shooter.blockShooter();
                shooterRunning = false;
                shooter.setIntakeState(Shooter.IntakeState.INTAKING); // Start intake before path
                drive.followPathChain(paths.shootingToIntake1, true);
                break;
            case 3:
                // Intake area reached, stop intake and start gate path (intake OFF during gate path)
                shooter.setIntakeState(Shooter.IntakeState.HOLD);
                drive.followPathChain(paths.intake1ToGate, true);
                break;
            case 4:
                // Gate path complete, wait 2.5 seconds for balls to fall from rail
                gateWaitTimer.reset();
                break;
            case 5:
                // Wait complete, start path from gate to shooting position
                drive.followPathChain(paths.gateToShooting2, true);
                break;
            case 6:
                // Shooting done at gateToShooting2, start intake BEFORE path to mid-field
                shooter.blockShooter();
                shooterRunning = false;
                shooter.setIntakeState(Shooter.IntakeState.INTAKING); // Start intake before path
                drive.followPathChain(paths.shooting2toIntake2, true);
                break;
            case 7:
                // Mid-field reached, start path back to intake area
                shooter.setIntakeState(Shooter.IntakeState.HOLD);
                drive.followPathChain(paths.intake2toShooting3, true);
                break;
            case 8:
                // Shooting done at intake2toShooting3, start intake BEFORE path to far position
                shooter.blockShooter();
                shooterRunning = false;
                shooter.setIntakeState(Shooter.IntakeState.INTAKING); // Start intake before path
                drive.followPathChain(paths.shooting3toIntake3, true);
                break;
            case 9:
                // Far position reached, stop intake and start path back to shooting
                shooter.setIntakeState(Shooter.IntakeState.HOLD);
                drive.followPathChain(paths.intake3ToShooting4, true);
                break;
            case 10:
                // This state is skipped - shooting handled in case 9 of autonomousPathUpdate
                break;
            case 11:
                // This state is skipped - not used
                break;
            case 12:
                // Shooting done, start final path to end position
                shooter.blockShooter();
                shooterRunning = false;
                shooter.setIntakeState(Shooter.IntakeState.HOLD);
                drive.followPathChain(paths.shootingToEnd, true);
                break;
            case 13:
                // Autonomous complete
                shooterRunning = false;
                shooter.setIntakeState(Shooter.IntakeState.HOLD);
                drive.stop();
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
                // Following startToShooting1
                if (!drive.isBusy()) {
                    // Path ends at optimal heading, limelight will fine-tune during shooting
                    setPathState(1); // End of startToShooting1 - shoot (first set, always timer-based)
                }
                break;
            case 1:
                // Shooting after startToShooting1 - shoot and align
                // Use shootAllThreeBalls() for consistent shooting with alignment
                try {
                    shootAllThreeBalls();
                } catch (InterruptedException e) {
                    // Handle interruption
                }
                setPathState(2); // Continue to next path
                break;
            case 2:
                // Following shootingToIntake1 (intake is ON during this path)
                if (!drive.isBusy()) {
                    // Stop intake when path to intake area is complete
                    shooter.setIntakeState(Shooter.IntakeState.HOLD);
                    setPathState(3); // Start intake1ToGate
                }
                break;
            case 3:
                // Following intake1ToGate (intake is OFF during this path)
                if (!drive.isBusy()) {
                    setPathState(4); // Start gate wait
                }
                break;
            case 4:
                // Wait 2.5 seconds after gate path for balls to fall from rail
                if (gateWaitTimer.seconds() >= GATE_WAIT_TIME) {
                    setPathState(5); // Start gateToShooting2
                }
                break;
            case 5:
                // Following gateToShooting2
                if (!drive.isBusy()) {
                    // Path ends at optimal heading, limelight will fine-tune during shooting
                    // Shoot all three balls (alignment happens in parallel)
                    try {
                        shootAllThreeBalls();
                    } catch (InterruptedException e) {
                        // Handle interruption
                    }
                    setPathState(6); // Continue to next path
                }
                break;
            case 6:
                // Following shooting2toIntake2 (intake is ON during this path)
                if (!drive.isBusy()) {
                    // Stop intake when path to intake area is complete
                    shooter.setIntakeState(Shooter.IntakeState.HOLD);
                    setPathState(7); // Start intake2toShooting3
                }
                break;
            case 7:
                // Following intake2toShooting3
                if (!drive.isBusy()) {
                    // Path ends at optimal heading, limelight will fine-tune during shooting
                    // Shoot all three balls (alignment happens in parallel)
                    try {
                        shootAllThreeBalls();
                    } catch (InterruptedException e) {
                        // Handle interruption
                    }
                    setPathState(8); // Continue to next path (start intake and follow shooting3toIntake3)
                }
                break;
            case 8:
                // Following shooting3toIntake3 (intake is ON during this path)
                if (!drive.isBusy()) {
                    // Stop intake when path to intake area is complete
                    shooter.setIntakeState(Shooter.IntakeState.HOLD);
                    setPathState(9); // Start intake3ToShooting4
                }
                break;
            case 9:
                // Following intake3ToShooting4
                if (!drive.isBusy()) {
                    // Path ends at optimal heading, limelight will fine-tune during shooting
                    // Shoot all three balls (alignment happens in parallel)
                    try {
                        shootAllThreeBalls();
                    } catch (InterruptedException e) {
                        // Handle interruption
                    }
                    setPathState(12); // Continue to next path (shootingToEnd)
                }
                break;
            case 10:
                // This state is skipped - not used
                break;
            case 11:
                // This state is skipped - not used
                break;
            case 12:
                // Following shootingToEnd
                if (!drive.isBusy()) {
                    setPathState(13); // Autonomous complete
                }
                break;
            case 13:
                // Autonomous complete
                break;
        }
    }
}
