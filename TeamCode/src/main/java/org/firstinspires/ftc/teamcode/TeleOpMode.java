package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;

import java.util.function.Supplier;

@TeleOp
public class TeleOpMode extends LinearOpMode{
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    //private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private boolean slowMode = false;

    @Override
    public void runOpMode() throws InterruptedException {
        //Run at Initialization
        Robot robot = new Robot(hardwareMap, telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();

        waitForStart();
        //On Start

        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robot.update();

            //Call this once per loop - Pedro Teleop
            follower.update();
            telemetry.update();
            //if (!automatedDrive) {
                //Make the last parameter false for field-centric
                //In case the drivers want to use a "slowMode" you can scale the vectors
                //This is the normal version to use in the TeleOp
                if (!gamepad1.right_bumper) follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true // Robot Centric
                );
                    //This is how it looks with slowMode on
                else follower.setTeleOpDrive(
                        -gamepad1.left_stick_y * robot.driveSpeedSlow,
                        -gamepad1.left_stick_x * robot.driveSpeedSlow,
                        -gamepad1.right_stick_x * robot.driveSpeedSlow,
                        true // Robot Centric
                );
            //}if(gamepad1.y){robot.outtakeByCode();}
            ////            if(gamepad1.x){robot.outtake(1);}//Purple
            ////            if(gamepad1.b){robot.outtake(2);}//Green
            //robot.driveWithControllers(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_bumper);

//

            if(gamepad1.dpad_up){if(robot.facingGoal!=true){robot.facingGoal = true;}else{robot.facingGoal = false;}}

            if (gamepad1.right_bumper && !robot.incremented){
                robot.flySpeed += robot.flySpeedIncre;
                robot.incremented = true;
            } else if (robot.triggerAsButton(gamepad1.right_trigger) && !robot.incremented){
                robot.flySpeed += robot.flySpeedIncre;
                robot.incremented = true;
            }
        }
    }
}