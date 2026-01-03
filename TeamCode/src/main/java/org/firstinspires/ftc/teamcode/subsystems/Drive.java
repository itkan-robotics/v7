package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import java.util.List;

/**
 * Drive subsystem wrapping Pedro Pathing Follower
 */
public class Drive {

    private Follower follower;
    private Pose2D startingPose;
    private List<LynxModule> allHubs;
    private ElapsedTime loopTimer = new ElapsedTime();
    private double lastLoopTimeMs = 0;

    public Drive(HardwareMap hardwareMap) {
        // Enable bulk reading for all hubs - HUGE loop time improvement!
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

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
    public void setMaxPower(double power){
        follower.setMaxPower(power);
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

    /**
     * Get the current pose of the robot
     */
    public Pose2D getCurrentPose() {
        Pose pose = follower.getPose();
        return new Pose2D(
                DistanceUnit.INCH, pose.getX(), pose.getY(),
                AngleUnit.RADIANS, pose.getHeading()
        );
    }

    /**
     * Call at the START of each loop iteration to begin timing
     */
    public void startLoopTimer() {
        loopTimer.reset();
    }

    /**
     * Call at the END of each loop iteration to record loop time
     * @return Loop time in milliseconds
     */
    public double endLoopTimer() {
        lastLoopTimeMs = loopTimer.milliseconds();
        return lastLoopTimeMs;
    }

    /**
     * Get the last recorded loop time
     * @return Loop time in milliseconds (target: <20ms)
     */
    public double getLoopTimeMs() {
        return lastLoopTimeMs;
    }

    /**
     * Clear bulk cache manually - only needed if using MANUAL caching mode
     * Call this at the start of each loop if you switch to MANUAL mode
     */
    public void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}




