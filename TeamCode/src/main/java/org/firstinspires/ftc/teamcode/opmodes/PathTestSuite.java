package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Test suite with selectable opmodes for testing different path types:
 * 1. Bezier Curve with heading changes
 * 2. Curve Heading Comparison - compares tangential vs linear heading interpolation
 * 3. Bezier Line strafing
 * 4. Bezier Line driving straight
 * 
 * Each opmode runs a path forward, waits 1 second, then runs the same path in reverse.
 */
@Configurable
@TeleOp(name = "Path Test Suite", group = "Testing")
public class PathTestSuite extends SelectableOpMode {
    public static Follower follower;
    private static TelemetryManager telemetryM;

    public PathTestSuite() {
        super("Select a Path Test", s -> {
            s.add("Bezier Curve Test", BezierCurveTest::new);
            s.add("Curve Heading Comparison", CurveHeadingComparisonTest::new);
            s.add("Strafe Line Test", StrafeLineTest::new);
            s.add("Straight Line Test", StraightLineTest::new);
        });
    }

    @Override
    public void onSelect() {
        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
        } else {
            follower = Constants.createFollower(hardwareMap);
        }

        follower.setStartingPose(new Pose(72, 72, Math.toRadians(0)));
        follower.activateAllPIDFs();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void onLog(java.util.List<String> lines) {}

    /**
     * OpMode 1: Bezier Curve with heading changes
     * Tests curved path following with heading interpolation
     */
    @Autonomous(name = "Bezier Curve Test", group = "Path Tests")
    public static class BezierCurveTest extends OpMode {
        private Path forwardPath;
        private Path reversePath;
        private ElapsedTime waitTimer = new ElapsedTime();
        private int state = 0; // 0 = forward, 1 = waiting, 2 = reverse, 3 = done

        @Override
        public void init() {
            // Create a curved path that changes heading
            forwardPath = new Path(new BezierCurve(
                    new Pose(72, 72, Math.toRadians(0)),
                    new Pose(100, 72, Math.toRadians(0)),
                    new Pose(100, 100, Math.toRadians(90))
            ));
            forwardPath.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90));

            // Create reverse path
            reversePath = new Path(new BezierCurve(
                    new Pose(100, 100, Math.toRadians(90)),
                    new Pose(100, 72, Math.toRadians(0)),
                    new Pose(72, 72, Math.toRadians(0))
            ));
            reversePath.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0));
            reversePath.reverseHeadingInterpolation();
        }

        @Override
        public void loop() {
            follower.update();

            switch (state) {
                case 0: // Start forward path
                    follower.followPath(forwardPath);
                    state = 1;
                    break;
                case 1: // Wait for forward path to complete
                    if (!follower.isBusy()) {
                        state = 2;
                        waitTimer.reset();
                    }
                    break;
                case 2: // Wait 1 second
                    if (waitTimer.seconds() >= 1.0) {
                        state = 3;
                    }
                    break;
                case 3: // Start reverse path
                    follower.followPath(reversePath);
                    state = 4;
                    break;
                case 4: // Wait for reverse path to complete
                    if (!follower.isBusy()) {
                        state = 5;
                    }
                    break;
                case 5: // Done
                    break;
            }

            telemetryM.debug("State: " + getStateName(state));
            telemetryM.debug("Path Busy: " + follower.isBusy());
            telemetryM.debug("X: " + String.format("%.2f", follower.getPose().getX()));
            telemetryM.debug("Y: " + String.format("%.2f", follower.getPose().getY()));
            telemetryM.debug("Heading: " + String.format("%.1f", Math.toDegrees(follower.getPose().getHeading())));
            telemetryM.update(telemetry);
        }

        private String getStateName(int state) {
            switch (state) {
                case 0: return "Starting Forward";
                case 1: return "Running Forward";
                case 2: return "Waiting (1s)";
                case 3: return "Starting Reverse";
                case 4: return "Running Reverse";
                case 5: return "Done";
                default: return "Unknown";
            }
        }
    }

    /**
     * OpMode 2: Curve Heading Comparison Test
     * Tests two curves - one with tangential heading and one with linear heading interpolation
     */
    @Autonomous(name = "Curve Heading Comparison", group = "Path Tests")
    public static class CurveHeadingComparisonTest extends OpMode {
        private Path tangentialCurveForward;
        private Path tangentialCurveReverse;
        private Path linearCurveForward;
        private Path linearCurveReverse;
        private ElapsedTime waitTimer = new ElapsedTime();
        private int state = 0; // 0 = tangential forward, 1 = waiting, 2 = tangential reverse, 3 = waiting, 4 = linear forward, 5 = waiting, 6 = linear reverse, 7 = done

        @Override
        public void init() {
            // Create a curve with tangential heading interpolation (forward)
            tangentialCurveForward = new Path(new BezierCurve(
                    new Pose(72, 72, Math.toRadians(0)),
                    new Pose(100, 72, Math.toRadians(0)),
                    new Pose(100, 100, Math.toRadians(90))
            ));
            tangentialCurveForward.setTangentHeadingInterpolation();

            // Create reverse path for tangential curve
            tangentialCurveReverse = new Path(new BezierCurve(
                    new Pose(100, 100, Math.toRadians(90)),
                    new Pose(100, 72, Math.toRadians(0)),
                    new Pose(72, 72, Math.toRadians(0))
            ));
            tangentialCurveReverse.setTangentHeadingInterpolation();
            tangentialCurveReverse.reverseHeadingInterpolation();

            // Create a curve with linear heading interpolation (forward)
            linearCurveForward = new Path(new BezierCurve(
                    new Pose(72, 72, Math.toRadians(0)),
                    new Pose(100, 72, Math.toRadians(0)),
                    new Pose(100, 100, Math.toRadians(90))
            ));
            linearCurveForward.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90));

            // Create reverse path for linear curve
            linearCurveReverse = new Path(new BezierCurve(
                    new Pose(100, 100, Math.toRadians(90)),
                    new Pose(100, 72, Math.toRadians(0)),
                    new Pose(72, 72, Math.toRadians(0))
            ));
            linearCurveReverse.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0));
        }

        @Override
        public void loop() {
            follower.update();

            switch (state) {
                case 0: // Start tangential curve forward
                    follower.followPath(tangentialCurveForward);
                    state = 1;
                    break;
                case 1: // Wait for tangential forward to complete
                    if (!follower.isBusy()) {
                        state = 2;
                        waitTimer.reset();
                    }
                    break;
                case 2: // Wait 1 second
                    if (waitTimer.seconds() >= 1.0) {
                        state = 3;
                    }
                    break;
                case 3: // Start tangential curve reverse
                    follower.followPath(tangentialCurveReverse);
                    state = 4;
                    break;
                case 4: // Wait for tangential reverse to complete
                    if (!follower.isBusy()) {
                        state = 5;
                        waitTimer.reset();
                    }
                    break;
                case 5: // Wait 1 second
                    if (waitTimer.seconds() >= 1.0) {
                        state = 6;
                    }
                    break;
                case 6: // Start linear curve forward
                    follower.followPath(linearCurveForward);
                    state = 7;
                    break;
                case 7: // Wait for linear forward to complete
                    if (!follower.isBusy()) {
                        state = 8;
                        waitTimer.reset();
                    }
                    break;
                case 8: // Wait 1 second
                    if (waitTimer.seconds() >= 1.0) {
                        state = 9;
                    }
                    break;
                case 9: // Start linear curve reverse
                    follower.followPath(linearCurveReverse);
                    state = 10;
                    break;
                case 10: // Wait for linear reverse to complete
                    if (!follower.isBusy()) {
                        state = 11;
                    }
                    break;
                case 11: // Done
                    break;
            }

            telemetryM.debug("State: " + getStateName(state));
            telemetryM.debug("Path Busy: " + follower.isBusy());
            telemetryM.debug("X: " + String.format("%.2f", follower.getPose().getX()));
            telemetryM.debug("Y: " + String.format("%.2f", follower.getPose().getY()));
            telemetryM.debug("Heading: " + String.format("%.1f", Math.toDegrees(follower.getPose().getHeading())));
            telemetryM.update(telemetry);
        }

        private String getStateName(int state) {
            switch (state) {
                case 0: return "Starting Tangential Forward";
                case 1: return "Running Tangential Forward";
                case 2: return "Waiting (1s)";
                case 3: return "Starting Tangential Reverse";
                case 4: return "Running Tangential Reverse";
                case 5: return "Waiting (1s)";
                case 6: return "Starting Linear Forward";
                case 7: return "Running Linear Forward";
                case 8: return "Waiting (1s)";
                case 9: return "Starting Linear Reverse";
                case 10: return "Running Linear Reverse";
                case 11: return "Done";
                default: return "Unknown";
            }
        }
    }

    /**
     * OpMode 3: Straight Line Test (moved after strafe)
     * Tests straight forward/backward movement
     */
    @Autonomous(name = "Straight Line Test", group = "Path Tests")
    public static class StraightLineTest extends OpMode {
        private Path forwardPath;
        private Path reversePath;
        private ElapsedTime waitTimer = new ElapsedTime();
        private int state = 0; // 0 = forward, 1 = waiting, 2 = reverse, 3 = done

        @Override
        public void init() {
            // Create a straight line path forward
            forwardPath = new Path(new BezierLine(
                    new Pose(72, 72, Math.toRadians(0)),
                    new Pose(72, 120, Math.toRadians(0))
            ));
            forwardPath.setConstantHeadingInterpolation(Math.toRadians(0));

            // Create reverse path
            reversePath = new Path(new BezierLine(
                    new Pose(72, 120, Math.toRadians(0)),
                    new Pose(72, 72, Math.toRadians(0))
            ));
            reversePath.setConstantHeadingInterpolation(Math.toRadians(0));
        }

        @Override
        public void loop() {
            follower.update();

            switch (state) {
                case 0: // Start forward path
                    follower.followPath(forwardPath);
                    state = 1;
                    break;
                case 1: // Wait for forward path to complete
                    if (!follower.isBusy()) {
                        state = 2;
                        waitTimer.reset();
                    }
                    break;
                case 2: // Wait 1 second
                    if (waitTimer.seconds() >= 1.0) {
                        state = 3;
                    }
                    break;
                case 3: // Start reverse path
                    follower.followPath(reversePath);
                    state = 4;
                    break;
                case 4: // Wait for reverse path to complete
                    if (!follower.isBusy()) {
                        state = 5;
                    }
                    break;
                case 5: // Done
                    break;
            }

            telemetryM.debug("State: " + getStateName(state));
            telemetryM.debug("Path Busy: " + follower.isBusy());
            telemetryM.debug("X: " + String.format("%.2f", follower.getPose().getX()));
            telemetryM.debug("Y: " + String.format("%.2f", follower.getPose().getY()));
            telemetryM.debug("Heading: " + String.format("%.1f", Math.toDegrees(follower.getPose().getHeading())));
            telemetryM.update(telemetry);
        }

        private String getStateName(int state) {
            switch (state) {
                case 0: return "Starting Forward";
                case 1: return "Running Forward";
                case 2: return "Waiting (1s)";
                case 3: return "Starting Reverse";
                case 4: return "Running Reverse";
                case 5: return "Done";
                default: return "Unknown";
            }
        }
    }

    /**
     * OpMode 3: Strafe Line Test
     * Tests lateral (strafe) movement
     */
    @Autonomous(name = "Strafe Line Test", group = "Path Tests")
    public static class StrafeLineTest extends OpMode {
        private Path forwardPath;
        private Path reversePath;
        private ElapsedTime waitTimer = new ElapsedTime();
        private int state = 0; // 0 = forward, 1 = waiting, 2 = reverse, 3 = done

        @Override
        public void init() {
            // Create a strafe path (moving in X direction, maintaining heading)
            forwardPath = new Path(new BezierLine(
                    new Pose(72, 72, Math.toRadians(0)),
                    new Pose(120, 72, Math.toRadians(0))
            ));
            forwardPath.setConstantHeadingInterpolation(Math.toRadians(0));

            // Create reverse path
            reversePath = new Path(new BezierLine(
                    new Pose(120, 72, Math.toRadians(0)),
                    new Pose(72, 72, Math.toRadians(0))
            ));
            reversePath.setConstantHeadingInterpolation(Math.toRadians(0));
        }

        @Override
        public void loop() {
            follower.update();

            switch (state) {
                case 0: // Start forward path
                    follower.followPath(forwardPath);
                    state = 1;
                    break;
                case 1: // Wait for forward path to complete
                    if (!follower.isBusy()) {
                        state = 2;
                        waitTimer.reset();
                    }
                    break;
                case 2: // Wait 1 second
                    if (waitTimer.seconds() >= 1.0) {
                        state = 3;
                    }
                    break;
                case 3: // Start reverse path
                    follower.followPath(reversePath);
                    state = 4;
                    break;
                case 4: // Wait for reverse path to complete
                    if (!follower.isBusy()) {
                        state = 5;
                    }
                    break;
                case 5: // Done
                    break;
            }

            telemetryM.debug("State: " + getStateName(state));
            telemetryM.debug("Path Busy: " + follower.isBusy());
            telemetryM.debug("X: " + String.format("%.2f", follower.getPose().getX()));
            telemetryM.debug("Y: " + String.format("%.2f", follower.getPose().getY()));
            telemetryM.debug("Heading: " + String.format("%.1f", Math.toDegrees(follower.getPose().getHeading())));
            telemetryM.update(telemetry);
        }

        private String getStateName(int state) {
            switch (state) {
                case 0: return "Starting Forward";
                case 1: return "Running Forward";
                case 2: return "Waiting (1s)";
                case 3: return "Starting Reverse";
                case 4: return "Running Reverse";
                case 5: return "Done";
                default: return "Unknown";
            }
        }
    }
}

