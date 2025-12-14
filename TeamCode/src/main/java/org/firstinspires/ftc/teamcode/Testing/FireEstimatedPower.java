package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name = "Fire Power Estimate", group = "Tests")
public class FireEstimatedPower extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        //Run at Initialization
        Robot robot = new Robot(hardwareMap, telemetry);

        waitForStart();
        //On Start

        robot.estimatePower();
        telemetry.addLine("Power: " + robot.flySpeed);
        telemetry.update();
        robot.outtakeByCode(new int[]{1,2,1});

        if (isStopRequested()) return;

        while (opModeIsActive()) {

        }
    }
}