package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name = "Goal Tracking", group = "Tests")
public class GoalFollower extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        //Run at Initialization
        Robot robot = new Robot(hardwareMap, telemetry);
        robot.faceGoal(true);

        waitForStart();
        //On Start

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robot.estimatePower();

            if(gamepad1.xWasPressed()){robot.outtake(1);}
            if(gamepad1.bWasPressed()){robot.outtake(2);}

            if(gamepad1.xWasReleased()){robot.outtake(0);}
            if(gamepad1.bWasReleased()){robot.outtake(0);}

            //robot.flyPower(true);
            robot.flyPower(true);
            telemetry.addLine("Power: " + robot.flySpeed);
            telemetry.update();
        }
    }
}