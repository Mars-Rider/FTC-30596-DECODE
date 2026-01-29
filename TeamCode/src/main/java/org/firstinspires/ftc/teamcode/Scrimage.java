package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot.Globals;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Settings;

@TeleOp
public class Scrimage extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        //Run at Initialization
        boolean lastTrigger = false;
        
        Robot robot = new Robot(hardwareMap, telemetry);
        robot.startDrivetrain();

        //LEDChannel auxLEDs = robot.LEDs.addChannel("auxLEDs");

        waitForStart();
        //On Start
        if (Globals.code[0] == 0) {
            //robot.readFieldData();
        }

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robot.update();
            telemetry.update();

            robot.drivetrain.drive(robot.overide(gamepad1.left_stick_x,gamepad2.left_stick_x), -robot.overide(gamepad1.left_stick_y,gamepad2.left_stick_y), robot.overide(gamepad1.right_stick_x,gamepad2.right_stick_x));

            if(robot.overide(gamepad1.xWasPressed(),gamepad2.xWasPressed())){robot.outtake(1, true);}
            if(robot.overide(gamepad1.bWasPressed(),gamepad2.bWasPressed())){robot.outtake(2, true);}
            if(robot.overide(gamepad1.xWasReleased(),gamepad2.xWasReleased())){robot.outtake(1, false);}
            if(robot.overide(gamepad1.bWasReleased(),gamepad2.bWasReleased())){robot.outtake(2, false   );}


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
                Robot.flySpeed -= Settings.flySpeedIncre;
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
                Robot.flySpeed += Settings.flySpeedIncre;
                robot.adjustFlySpeed();
            } else if (robot.overide(robot.triggerAsButton(gamepad1.right_trigger),robot.triggerAsButton(gamepad2.right_trigger)) && !lastTrigger){
                Robot.flySpeed -= Settings.flySpeedIncre;
                robot.adjustFlySpeed();
                lastTrigger = true;
            } else if (!robot.overide(robot.triggerAsButton(gamepad1.right_trigger),robot.triggerAsButton(gamepad2.right_trigger))){
                lastTrigger = false;
            }
            robot.adjustFlySpeed();

            telemetry.addLine("Fly Wheel Speed: "+ Robot.flySpeed);
            telemetry.addLine("Sort On: "+ robot.sortOn);
            telemetry.addLine("fly encoder: "+ robot.bFly.getVelocity(AngleUnit.RADIANS));
        }
    }
}