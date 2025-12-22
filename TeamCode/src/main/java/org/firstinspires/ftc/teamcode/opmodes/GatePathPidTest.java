package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/**
 * PID Tuning Test - Runs the exact same paths as GatePathNew without any other hardware.
 * Configurable wait time between paths (0-2000ms) using gamepad during init.
 */
@Autonomous(name = "Gate Path PID Test", group = "Testing")
public class GatePathPidTest extends LinearOpMode {
    private TelemetryManager panelsTelemetry;
    private Drive drive;
    
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    // Timer for waiting between paths
    private ElapsedTime waitTimer = new ElapsedTime();
    
    // Configurable wait time between paths (milliseconds)
    // Range: 0-2000ms, default: 500ms
    // Adjust with gamepad during init: DPad Up/Down or Right Bumper/Trigger
    private int waitTimeMs = 500;
    
    // Gamepad control state for debouncing
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastRightBumper = false;
    private boolean lastRightTrigger = false;
    
    // Starting pose - MUST match the beginning of Path1!
    private final Pose2D startPose = new Pose2D(DistanceUnit.INCH, 128.72445019404915, 113.26261319534282, AngleUnit.RADIANS, Math.toRadians(-90));

    @Override
    public void runOpMode() throws InterruptedException {
        // === INIT ===
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        drive = new Drive(hardwareMap);
        drive.setStartingPose(startPose);
        paths = new Paths(drive.getFollower());
        pathState = 0;

        // === INIT LOOP - Adjust wait time with gamepad ===
        while (!isStarted() && !isStopRequested()) {
            // Gamepad controls for adjusting wait time
            // DPad Up or Right Bumper: Increase by 50ms
            // DPad Down or Right Trigger: Decrease by 50ms
            
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean rightBumper = gamepad1.right_bumper;
            boolean rightTrigger = gamepad1.right_trigger > 0.5;
            
            // Increment on button press (debounced)
            if ((dpadUp && !lastDpadUp) || (rightBumper && !lastRightBumper)) {
                waitTimeMs += 50;
                if (waitTimeMs > 2000) {
                    waitTimeMs = 2000;
                }
            }
            
            // Decrement on button press (debounced)
            if ((dpadDown && !lastDpadDown) || (rightTrigger && !lastRightTrigger)) {
                waitTimeMs -= 50;
                if (waitTimeMs < 0) {
                    waitTimeMs = 0;
                }
            }
            
            // Update last state for debouncing
            lastDpadUp = dpadUp;
            lastDpadDown = dpadDown;
            lastRightBumper = rightBumper;
            lastRightTrigger = rightTrigger;

            // Display telemetry
            telemetry.addLine("=== GATE PATH PID TEST ===");
            telemetry.addData("Wait Time Between Paths", waitTimeMs + " ms");
            telemetry.addLine("");
            telemetry.addLine("CONTROLS (during init):");
            telemetry.addLine("  DPad Up / Right Bumper: +50ms");
            telemetry.addLine("  DPad Down / Right Trigger: -50ms");
            telemetry.addLine("");
            telemetry.addLine("Ready to start - all paths will run sequentially");
            telemetry.update();
            panelsTelemetry.update(telemetry);
            sleep(50);
        }

        // === START ===
        setPathState(0);

        // === MAIN LOOP ===
        while (opModeIsActive()) {
            drive.update(); // Update Pedro Pathing
            autonomousPathUpdate(); // Update autonomous state machine

            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("Current Path", getPathName(pathState));
            Pose2D currentPose = drive.getCurrentPose();
            panelsTelemetry.debug("X", currentPose.getX(DistanceUnit.INCH));
            panelsTelemetry.debug("Y", currentPose.getY(DistanceUnit.INCH));
            panelsTelemetry.debug("Heading", currentPose.getHeading(AngleUnit.RADIANS));
            panelsTelemetry.debug("Wait Time (ms)", waitTimeMs);
            panelsTelemetry.debug("Path Busy", drive.isBusy());
            panelsTelemetry.update(telemetry);
        }
    }

    /**
     * Paths class - exact same as GatePathNew
     */
    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(128.724, 113.263),
                                    new Pose(92.000, 88.000)
                            )
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
                                    new Pose(92.000, 60.000),
                                    new Pose(92.000, 88.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
                    
            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(92.000, 88.000),
                                    new Pose(92.000, 55.000),
                                    new Pose(116.000, 54.000),
                                    new Pose(132.75, 61.4)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(131.705, 60.916),
                                    new Pose(116.000, 54.000),
                                    new Pose(92.000, 53.000),
                                    new Pose(92.000, 88)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
                    
            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(92.000, 88.000),
                                    new Pose(83.000, 88.000),
                                    new Pose(120.155, 83.084)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120.155, 83.084), new Pose(92.000, 88.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(92.000, 88.000),
                                    new Pose(83.000, 43.000),
                                    new Pose(132.000, 62.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(132.000, 62.000),
                                    new Pose(83.000, 43.000),
                                    new Pose(92.000, 88.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(92.000, 88.000),
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
                            new BezierLine(new Pose(133.382, 23.472), new Pose(92.000, 88.000))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
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
                drive.followPathChain(paths.Path1, true);
                break;
            case 1:
                // Wait state - handled in autonomousPathUpdate
                waitTimer.reset();
                break;
            case 2:
                drive.followPathChain(paths.Path2, true);
                break;
            case 3:
                waitTimer.reset();
                break;
            case 4:
                drive.followPathChain(paths.Path3, true);
                break;
            case 5:
                waitTimer.reset();
                break;
            case 6:
                drive.followPathChain(paths.Path4, true);
                break;
            case 7:
                waitTimer.reset();
                break;
            case 8:
                drive.followPathChain(paths.Path5, true);
                break;
            case 9:
                waitTimer.reset();
                break;
            case 10:
                drive.followPathChain(paths.Path6, true);
                break;
            case 11:
                waitTimer.reset();
                break;
            case 12:
                drive.followPathChain(paths.Path7, true);
                break;
            case 13:
                waitTimer.reset();
                break;
            case 14:
                drive.followPathChain(paths.Path8, true);
                break;
            case 15:
                waitTimer.reset();
                break;
            case 16:
                drive.followPathChain(paths.Path9, true);
                break;
            case 17:
                waitTimer.reset();
                break;
            case 18:
                drive.followPathChain(paths.Path10, true);
                break;
            case 19:
                waitTimer.reset();
                break;
            case 20:
                drive.followPathChain(paths.Path11, true);
                break;
            case 21:
                // All paths complete
                break;
        }
    }

    /**
     * State machine for autonomous path following.
     * Checks if the current path is complete and transitions to the next state.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Path1
                if (!drive.isBusy()) {
                    setPathState(1); // Go to wait state
                }
                break;
            case 1: // Wait after Path1
                if (waitTimer.milliseconds() >= waitTimeMs) {
                    setPathState(2); // Start Path2
                }
                break;
            case 2: // Path2
                if (!drive.isBusy()) {
                    setPathState(3); // Go to wait state
                }
                break;
            case 3: // Wait after Path2
                if (waitTimer.milliseconds() >= waitTimeMs) {
                    setPathState(4); // Start Path3
                }
                break;
            case 4: // Path3
                if (!drive.isBusy()) {
                    setPathState(5); // Go to wait state
                }
                break;
            case 5: // Wait after Path3
                if (waitTimer.milliseconds() >= waitTimeMs) {
                    setPathState(6); // Start Path4
                }
                break;
            case 6: // Path4
                if (!drive.isBusy()) {
                    setPathState(7); // Go to wait state
                }
                break;
            case 7: // Wait after Path4
                if (waitTimer.milliseconds() >= waitTimeMs) {
                    setPathState(8); // Start Path5
                }
                break;
            case 8: // Path5
                if (!drive.isBusy()) {
                    setPathState(9); // Go to wait state
                }
                break;
            case 9: // Wait after Path5
                if (waitTimer.milliseconds() >= waitTimeMs) {
                    setPathState(10); // Start Path6
                }
                break;
            case 10: // Path6
                if (!drive.isBusy()) {
                    setPathState(11); // Go to wait state
                }
                break;
            case 11: // Wait after Path6
                if (waitTimer.milliseconds() >= waitTimeMs) {
                    setPathState(12); // Start Path7
                }
                break;
            case 12: // Path7
                if (!drive.isBusy()) {
                    setPathState(13); // Go to wait state
                }
                break;
            case 13: // Wait after Path7
                if (waitTimer.milliseconds() >= waitTimeMs) {
                    setPathState(14); // Start Path8
                }
                break;
            case 14: // Path8
                if (!drive.isBusy()) {
                    setPathState(15); // Go to wait state
                }
                break;
            case 15: // Wait after Path8
                if (waitTimer.milliseconds() >= waitTimeMs) {
                    setPathState(16); // Start Path9
                }
                break;
            case 16: // Path9
                if (!drive.isBusy()) {
                    setPathState(17); // Go to wait state
                }
                break;
            case 17: // Wait after Path9
                if (waitTimer.milliseconds() >= waitTimeMs) {
                    setPathState(18); // Start Path10
                }
                break;
            case 18: // Path10
                if (!drive.isBusy()) {
                    setPathState(19); // Go to wait state
                }
                break;
            case 19: // Wait after Path10
                if (waitTimer.milliseconds() >= waitTimeMs) {
                    setPathState(20); // Start Path11
                }
                break;
            case 20: // Path11
                if (!drive.isBusy()) {
                    setPathState(21); // All paths complete
                }
                break;
            case 21: // Done - all paths complete
                // Stay in this state
                break;
        }
    }

    /**
     * Get the name of the current path for telemetry
     */
    private String getPathName(int state) {
        if (state == 0) return "Path1";
        if (state == 2) return "Path2";
        if (state == 4) return "Path3";
        if (state == 6) return "Path4";
        if (state == 8) return "Path5";
        if (state == 10) return "Path6";
        if (state == 12) return "Path7";
        if (state == 14) return "Path8";
        if (state == 16) return "Path9";
        if (state == 18) return "Path10";
        if (state == 20) return "Path11";
        if (state == 21) return "Complete";
        if (state % 2 == 1) return "Waiting (" + waitTimeMs + "ms)";
        return "Unknown";
    }
}

