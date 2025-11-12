package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class PowerEstimate extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        //Run at Initialization
        Robot robot = new Robot(hardwareMap, telemetry);

        waitForStart();
        //On Start

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robot.estimatePower();
            //robot.flyPower(true);
            robot.tFly.setPower(robot.flySpeed);
            telemetry.addLine("Power: " + robot.flySpeed);
            telemetry.update();
        }
    }
}