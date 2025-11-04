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
public class Scrimage extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        //Run at Initialization
        Robot robot = new Robot(hardwareMap, telemetry);

        waitForStart();
        //On Start



        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robot.update();
            telemetry.update();

            robot.SwerveDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            //if(gamepad1.dpad_up){if(robot.facingGoal!=true){robot.facingGoal = true;}else{robot.facingGoal = false;}}


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