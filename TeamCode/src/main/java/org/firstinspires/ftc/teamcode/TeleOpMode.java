package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Globals;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Settings;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class TeleOpMode extends OpMode {
    boolean lastTrigger = false;
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap,telemetry);

        robot.startPedroDrivetrain();
        robot.drivetrain.startingPose = Globals.startPose;
    }
    @Override
    public void start() {
        robot.drivetrain.begin();
    }
    @Override
    public void loop() {
        robot.update();
        robot.drivetrain.drive(robot.overide(gamepad1.left_stick_x,gamepad2.left_stick_x), robot.overide(gamepad1.left_stick_y,gamepad2.left_stick_y), robot.overide(gamepad1.right_stick_x,gamepad2.right_stick_x));

        if(robot.overide(gamepad1.xWasPressed(),gamepad2.xWasPressed())){robot.outtake(1);}
        if(robot.overide(gamepad1.bWasPressed(),gamepad2.bWasPressed())){robot.outtake(2);}

        if(robot.overide(gamepad1.aWasPressed(),gamepad2.aWasPressed())){robot.flyPower();}
        if(robot.overide(gamepad1.yWasPressed(),gamepad2.yWasPressed())){robot.intakePower();}

        if(robot.overide(gamepad1.dpad_left, gamepad2.dpad_left)){
            robot.sort(2);
        } else if (robot.overide(gamepad1.dpad_right, gamepad2.dpad_right)){
            robot.sort(1);
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

        if (robot.overide(gamepad1.leftStickButtonWasPressed(),gamepad2.leftStickButtonWasPressed())){
            //robot.drivetrain.runTo(new Pose(robot.changeAlliance(40), 90, Globals.alliance == 1 ? Math.toRadians(-45) : Math.toRadians(-135)), robot.faceGoalError(false));
        }

        if (robot.overide(gamepad1.rightStickButtonWasPressed(),gamepad2.rightStickButtonWasPressed())){
            //Face Goal rn
            robot.outtakeByCode();
        }

        if (robot.overide(robot.triggerAsButton(gamepad1.left_trigger),robot.triggerAsButton(gamepad2.left_trigger))){
            //Face Goal rn
            robot.drivetrain.runTo(robot.faceGoalError());
        }

        if (robot.overide(gamepad1.dpadUpWasPressed(),gamepad2.dpadUpWasPressed())){
            //Face Goal rn
            robot.estimatePower();
            robot.adjustFlySpeed();
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
        telemetry.addLine("Goal Error to Field: "+ Math.toDegrees(robot.faceGoalError()));
        telemetry.addLine("Robot Heading: "+ Math.toDegrees(robot.drivetrain.getHeading()));

        //robot.estimatePower();
    }
}