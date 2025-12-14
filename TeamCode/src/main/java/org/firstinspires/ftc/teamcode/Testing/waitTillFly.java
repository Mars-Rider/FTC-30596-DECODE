package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name = "Wait Till", group = "Tests")
public class waitTillFly extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        //Run at Initialization
        Robot robot = new Robot(hardwareMap, telemetry);

        waitForStart();
        //On Start

        robot.flyPower(true);
        robot.sleepForFly();
        robot.outtake(1);
        robot.outtake(2);

        if (isStopRequested()) return;

        while (opModeIsActive()) {

        }
    }
}