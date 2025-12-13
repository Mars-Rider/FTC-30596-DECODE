package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Globals;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Settings;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;

import java.util.function.Supplier;

@TeleOp
public class TeleOpMode extends LinearOpMode{
    boolean lastTrigger = false;

    @Override
    public void runOpMode() throws InterruptedException {
        //Run at Initialization
        Robot robot = new Robot(hardwareMap, telemetry);
        robot.startPedroDrivetrain();
        robot.drivetrain.startingPose = Globals.startPose;

        waitForStart();
        //On Start

        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robot.update();

            if(robot.overide(gamepad1.xWasPressed(),gamepad2.xWasPressed())){robot.outtake(1);}
            if(robot.overide(gamepad1.bWasPressed(),gamepad2.bWasPressed())){robot.outtake(2);}

            //if(robot.overide(gamepad1.xWasReleased(),gamepad2.xWasReleased())){robot.outtake(0);}
            //if(robot.overide(gamepad1.bWasReleased(),gamepad2.bWasReleased())){robot.outtake(0);}

            if(robot.overide(gamepad1.aWasPressed(),gamepad2.aWasPressed())){robot.flyPower();}
            if(robot.overide(gamepad1.yWasPressed(),gamepad2.yWasPressed())){robot.intakePower();}

            if(robot.overide(gamepad1.dpad_left, gamepad2.dpad_left)){
                robot.sort(2);
            } else if (robot.overide(gamepad1.dpad_right, gamepad2.dpad_right)){
                robot.sort(1);
            } else if(robot.overide(gamepad1.dpadDownWasPressed(), gamepad2.dpadDownWasPressed())){
                robot.autoSorting();
            } else {
                robot.sort();
            }

            if(robot.overide(gamepad1.leftBumperWasPressed(), gamepad2.leftBumperWasPressed())){
                if(robot.intakeDirection != 1){
                    robot.intakeDirection = 1;
                } else {
                    robot.intakeDirection = -1;
                }
            }

            if(gamepad1.leftBumperWasPressed()){
                //robot.drivetrain.slowMode();
            }

            if (robot.overide(robot.triggerAsButton(gamepad1.right_trigger),robot.triggerAsButton(gamepad2.right_trigger)) && !lastTrigger){
                robot.flySpeed -= Settings.flySpeedIncre;
                robot.adjustFlySpeed();
                lastTrigger = true;
            } else if (!robot.overide(robot.triggerAsButton(gamepad1.right_trigger),robot.triggerAsButton(gamepad2.right_trigger))){
                lastTrigger = false;
            }

            if (gamepad1.dpad_down){
                //Face Goal rn
                //robot.drivetrain.runTo(robot.faceGoalError(false));
            }

            if (robot.overide(gamepad1.rightBumperWasPressed(), gamepad2.rightBumperWasPressed())){
                robot.flySpeed += Settings.flySpeedIncre;
                robot.adjustFlySpeed();
            } else if (robot.overide(robot.triggerAsButton(gamepad1.right_trigger),robot.triggerAsButton(gamepad2.right_trigger)) && !lastTrigger){
                robot.flySpeed -= Settings.flySpeedIncre;
                robot.adjustFlySpeed();
                lastTrigger = true;
            } else if (!robot.overide(robot.triggerAsButton(gamepad1.right_trigger),robot.triggerAsButton(gamepad2.right_trigger))){
                lastTrigger = false;
            }
            telemetry.addLine("Fly Wheel Speed: "+ robot.flySpeed);
            telemetry.addLine("Sort On: "+ robot.sortOn);
            telemetry.addLine("fly encoder: "+ robot.bFly.getVelocity(AngleUnit.RADIANS));
        }
    }
}