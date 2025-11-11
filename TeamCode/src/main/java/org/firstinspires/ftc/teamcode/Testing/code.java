package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.Arrays;

@Autonomous
public class code extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        //Run at Initialization

        waitForStart();
        //On Start
        telemetry.addLine("Code: " + Arrays.toString(Globals.code));
        telemetry.addLine("alliance: " + Globals.alliance);

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.update();
        }
    }
}