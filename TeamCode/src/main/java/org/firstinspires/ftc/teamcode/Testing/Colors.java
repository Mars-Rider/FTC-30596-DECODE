package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name = "Colors", group = "Tests")
public class Colors extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        //Run at Initialization
        Robot robot = new Robot(hardwareMap, telemetry);

        waitForStart();
        //On Start

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addLine("P Intake Sensor: " + robot.color(robot.pColor[0]));
            telemetry.addLine("P Outake Sensor: " + robot.color(robot.pColor[1]));

            telemetry.addLine("G Intake Sensor: " + robot.color(robot.gColor[0]));
            telemetry.addLine("G Outtake Sensor: " + robot.color(robot.gColor[1]));
            telemetry.update();
        }
    }
}