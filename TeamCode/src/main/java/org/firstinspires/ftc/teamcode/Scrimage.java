package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Scrimage extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        //Run at Initialization
        Robot robot = new Robot(hardwareMap, telemetry);

        waitForStart();
        //On Start
        if (Globals.code[0] == 0) {
            //robot.readFieldData();
        }

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robot.update();
            telemetry.update();

            robot.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if(gamepad1.xWasPressed()){robot.outtake(1);}
            if(gamepad1.bWasPressed()){robot.outtake(2);}

            if(gamepad1.xWasReleased()){robot.outtake(0);}
            if(gamepad1.bWasReleased()){robot.outtake(0);}

            if(gamepad1.aWasPressed()){robot.flyPower();}
            if(gamepad1.yWasPressed()){robot.intakePower();}

            //if(gamepad1.aWasReleased()){robot.tFly.setPower(0);}
            //if(gamepad1.yWasReleased()){robot.intake.setPower(0);}

            if(gamepad1.dpad_left){
                robot.sort(2);
            } else if (gamepad1.dpad_right){
                robot.sort(1);
            } else if(gamepad1.dpadDownWasPressed()){
                if(robot.sortOn){
                    robot.sortOn = false;
                } else {
                    robot.sortOn = true;
                }
            } else {
                if(robot.sortOn){
                    robot.sort();
                } else {
                    robot.sort(0);
                }
            }

            if(gamepad1.dpad_up){
                robot.estimatePower();
            }

            if (gamepad1.dpad_down){
                //robot.outtakeByCode();
            }

            if (gamepad1.rightBumperWasPressed()){
                robot.flySpeed += robot.flySpeedIncre;
                robot.adjustFlySpeed();
            } else if (robot.triggerAsButtonPress(gamepad1.right_trigger) && !robot.incremented){
                robot.flySpeed -= robot.flySpeedIncre;
                robot.adjustFlySpeed();
            }

            telemetry.addLine("Fly Wheel Speed: "+ robot.flySpeed);
            telemetry.addLine("Sort On: "+ robot.sortOn);
        }
    }
}