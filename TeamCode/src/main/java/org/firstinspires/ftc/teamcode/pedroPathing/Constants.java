package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.11092)
            .forwardZeroPowerAcceleration(-35.436696126559625)
            .lateralZeroPowerAcceleration(-95.87296953434631)
            .useSecondaryDrivePIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(10, 0, 0, 0.6, 0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.001, 0, 0.002, 0.6, 0.025))
            .translationalPIDFCoefficients(new PIDFCoefficients(1, 0, 0, 0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.0085, 0, 0.04, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7,0,0,0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.65, 0, 0.1, 0.001))
            .centripetalScaling(0.0006);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftRearMotorName("back_left")
            .leftFrontMotorName("front_left")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(91.35502948911173)
            .yVelocity(64.44296577033097);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.95,
            100,
            0.935,
            1);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-55)
            .strafePodX(-12.5)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .customEncoderResolution(34.311)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
