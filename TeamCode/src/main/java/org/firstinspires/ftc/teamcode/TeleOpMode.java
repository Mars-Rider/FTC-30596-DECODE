package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class TeleOpMode extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        //Run at Initialization
        Robot robot = new Robot(hardwareMap, telemetry);


        waitForStart();
        //On Start


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robot.loop();
            robot.driveWithControllers(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.right_bumper);

            if(gamepad1.y){robot.outtakeByCode();}
            if(gamepad1.x){robot.outtake(1);}//Purple
            if(gamepad1.b){robot.outtake(2);}//Green

            if(gamepad1.dpad_up){robot.faceGoal();}

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