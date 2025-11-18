package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Globals;
import org.firstinspires.ftc.teamcode.Robot.Robot;

import java.util.Arrays;

@TeleOp(name = "FieldData", group = "Tests")
public class FieldData extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        //Run at Initialization
        Robot robot = new Robot(hardwareMap, telemetry);

        waitForStart();
        //On Start
        robot.readFieldData();
        telemetry.addLine("Code: " + Arrays.toString(Globals.code));
        telemetry.addLine("alliance: " + Globals.alliance);

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.update();
        }
    }
}