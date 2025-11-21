package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Globals;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp
public class Scrimage extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        //Run at Initialization
        Robot robot = new Robot(hardwareMap, telemetry);
        robot.startDrivetrain();

        waitForStart();
        //On Start
        if (Globals.code[0] == 0) {
            //robot.readFieldData();
        }

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //robot.update();
            telemetry.update();

            robot.drivetrain.drive(robot.overide(-gamepad1.left_stick_x,-gamepad2.left_stick_x), robot.overide(gamepad1.left_stick_y,gamepad2.left_stick_y), robot.overide(gamepad1.right_stick_x,gamepad2.right_stick_x));

            if(robot.overide(gamepad1.xWasPressed(),gamepad2.xWasPressed())){robot.outtake(1);}
            if(robot.overide(gamepad1.bWasPressed(),gamepad2.bWasPressed())){robot.outtake(2);}

            if(robot.overide(gamepad1.xWasReleased(),gamepad2.xWasReleased())){robot.outtake(0);}
            if(robot.overide(gamepad1.bWasReleased(),gamepad2.bWasReleased())){robot.outtake(0);}

            if(robot.overide(gamepad1.aWasPressed(),gamepad2.aWasPressed())){robot.flyPower();}
            if(robot.overide(gamepad1.yWasPressed(),gamepad2.yWasPressed())){robot.intakePower();}

            //if(gamepad1.aWasReleased()){robot.tFly.setPower(0);}
            //if(gamepad1.yWasReleased()){robot.intake.setPower(0);}

            if(robot.overide(gamepad1.dpad_left, gamepad2.dpad_left)){
                robot.sort(2);
            } else if (robot.overide(gamepad1.dpad_right, gamepad2.dpad_right)){
                robot.sort(1);
            } else if(robot.overide(gamepad1.dpadDownWasPressed(), gamepad2.dpadLeftWasPressed())){
                robot.autoSorting();
            } else {
                robot.sort();
            }

            if(robot.overide(gamepad1.dpad_up, gamepad2.dpad_up)){
                robot.estimatePower();
            }

            if (gamepad1.dpad_down){
                //robot.outtakeByCode();
            }

            if (robot.overide(gamepad1.rightBumperWasPressed(), gamepad2.rightBumperWasPressed())){
                robot.flySpeed += robot.flySpeedIncre;
                robot.adjustFlySpeed();
            } else if (robot.overide(robot.triggerAsButtonPress(gamepad1.right_trigger),robot.triggerAsButtonPress(gamepad2.right_trigger))){
                robot.flySpeed -= robot.flySpeedIncre;
                robot.adjustFlySpeed();
            }
            telemetry.addLine("Fly Wheel Speed: "+ robot.flySpeed);
            telemetry.addLine("Sort On: "+ robot.sortOn);
        }
    }
}