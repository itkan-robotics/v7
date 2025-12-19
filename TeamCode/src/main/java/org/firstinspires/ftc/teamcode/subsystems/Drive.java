package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Drive subsystem that encapsulates drivetrain control and Pedro pathing integration.
 * Provides methods for manual teleop drive, path following, and pose tracking.
 * All position data is derived from the Pedro follower, which handles the pinpoint odometry module.
 */
public class Drive {
    private Follower follower;
    private boolean isTeleopMode = false;

    /**
     * Initialize the Drive subsystem with Pedro pathing.
     * The Pedro follower handles the pinpoint odometry module internally.
     * @param hardwareMap HardwareMap for accessing hardware
     */
    public Drive(HardwareMap hardwareMap) {
        // Create Pedro pathing follower (handles pinpoint odometry internally)
        follower = Constants.createFollower(hardwareMap);
    }

    /**
     * Start teleop drive mode. Must be called before using driveTeleop().
     */
    public void startTeleopDrive() {
        follower.startTeleopDrive();
        isTeleopMode = true;
    }

    /**
     * Drive the robot using teleop controls
     * @param x Forward/backward power (-1.0 to 1.0, positive = forward)
     * @param y Left/right strafe power (-1.0 to 1.0, positive = left)
     * @param rotation Rotation power (-1.0 to 1.0, positive = counterclockwise)
     * @param fieldRelative If true, drive relative to field coordinates. If false, drive relative to robot.
     */
    public void driveTeleop(double x, double y, double rotation, boolean fieldRelative) {
        if (!isTeleopMode) {
            startTeleopDrive();
        }
        follower.setTeleOpDrive(x, y, rotation, fieldRelative);
    }

    /**
     * Get the current robot pose from Pedro pathing follower.
     * The follower uses the pinpoint odometry module for localization.
     * @return Current pose as Pose2D
     */
    public Pose2D getCurrentPose() {
        Pose pose = follower.getPose();
        return new Pose2D(
                DistanceUnit.INCH,
                pose.getX(),
                pose.getY(),
                AngleUnit.RADIANS,
                pose.getHeading()
        );
    }

    /**
     * Drive to a target position using path following
     * @param target Target pose to drive to
     */
    public void driveToPosition(Pose2D target) {
        // Convert Pose2D to Pedro Pose
        Pose currentPose = follower.getPose();
        Pose targetPose = new Pose(
                target.getX(DistanceUnit.INCH),
                target.getY(DistanceUnit.INCH),
                target.getHeading(AngleUnit.RADIANS)
        );
        
        // Create a straight-line path to the target
        Path path = new Path(new BezierLine(currentPose, targetPose));
        path.setConstantHeadingInterpolation(targetPose.getHeading());
        
        // Activate all PIDFs for smooth path following
        follower.activateAllPIDFs();
        
        // Follow the path
        follower.followPath(path);
        isTeleopMode = false;
    }

    /**
     * Drive to a target position with a specific heading
     * @param targetX Target X coordinate in inches
     * @param targetY Target Y coordinate in inches
     * @param targetHeading Target heading in radians
     */
    public void driveToPosition(double targetX, double targetY, double targetHeading) {
        Pose2D target = new Pose2D(
                DistanceUnit.INCH,
                targetX,
                targetY,
                AngleUnit.RADIANS,
                targetHeading
        );
        driveToPosition(target);
    }

    /**
     * Check if the follower is currently busy following a path
     * @return true if following a path, false otherwise
     */
    public boolean isBusy() {
        return follower.isBusy();
    }

    /**
     * Stop all movement
     */
    public void stop() {
        follower.setTeleOpDrive(0, 0, 0, false);
        if (follower.isBusy()) {
            // Cancel current path if following one
            follower.breakFollowing();
        }
    }

    /**
     * Update the follower. Must be called every loop iteration.
     */
    public void update() {
        follower.update();
    }

    /**
     * Get the Pedro pathing Follower instance (for advanced usage)
     * @return The Follower instance
     */
    public Follower getFollower() {
        return follower;
    }

    /**
     * Set the starting pose for the follower (useful for initialization)
     * @param pose Starting pose
     */
    public void setStartingPose(Pose2D pose) {
        Pose pedroPose = new Pose(
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.RADIANS)
        );
        follower.setStartingPose(pedroPose);
    }

    /**
     * Follow a path chain
     * @param pathChain The path chain to follow
     * @param activatePIDFs Whether to activate all PIDFs
     */
    public void followPathChain(PathChain pathChain, boolean activatePIDFs) {
        if (activatePIDFs) {
            follower.activateAllPIDFs();
        }
        follower.followPath(pathChain);
        isTeleopMode = false;
    }


    /**
     * Gate Path autonomous paths (from GatePath.java)
     */
    public static class GatePaths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;

        public GatePaths(Follower follower) {
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
                                    new Pose(134.000, 60.000),
                                    new Pose(100.000, 64.000),
                                    new Pose(96.000, 96.000))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
        }
    }
}

