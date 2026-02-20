package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("LRL") // rfb
            .rightRearMotorName("LFB") // rrl
            .leftRearMotorName("RFB") // lrl
            .leftFrontMotorName("RRL") // lfb
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)//Strafe and
            .xVelocity(46.553280373812115)
            .yVelocity(67.22709803171244);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("LFB")
            .strafeEncoder_HardwareMapName("LRL")
            .forwardEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE)
            .forwardPodY(0.95)//Inch
            .strafePodX(3)//Inch
            .forwardTicksToInches(0.002983033994158225)
            .strafeTicksToInches(0.0029852602773804346)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            );

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.3)
            .forwardZeroPowerAcceleration(-35.240358568070064)
            .lateralZeroPowerAcceleration(-118.72354583964098)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.05))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.01, 0.05))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(1,0.0,0,0.6,0.1))
            .centripetalScaling(0.005);//In KG
    ;//In KG


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

}


