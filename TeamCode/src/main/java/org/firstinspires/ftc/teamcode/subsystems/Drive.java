package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/**
 * Drive subsystem wrapping Pedro Pathing Follower
 */
public class Drive {

    private Follower follower;
    private Pose2D startingPose;

    public Drive(HardwareMap hardwareMap) {
        // Initialize Pedro Pathing Follower using Constants builder
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
    }

    /**
     * Set the starting pose for the robot
     */
    public void setStartingPose(Pose2D pose) {
        this.startingPose = pose;
        follower.setStartingPose(new Pose(
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.RADIANS)
        ));
    }

    /**
     * Get the Follower instance for path building
     */
    public Follower getFollower() {
        return follower;
    }

    /**
     * Update the follower - call this every loop iteration
     */
    public void update() {
        follower.update();
    }

    /**
     * Start following a path chain
     */
    public void followPathChain(PathChain pathChain, boolean holdEnd) {
        follower.followPath(pathChain, holdEnd);
    }

    /**
     * Check if the robot is currently following a path
     */
    public boolean isBusy() {
        return follower.isBusy();
    }
}

