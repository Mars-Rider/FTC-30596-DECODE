package org.firstinspires.ftc.teamcode.Testing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import android.provider.Settings;

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
        Globals.opMode = true;
        Globals.alliance = 0;

        waitForStart();
        //On Start
        robot.readFieldData(true);
        telemetry.addLine("Code: " + Arrays.toString(Globals.code));
        telemetry.addLine("alliance: " + Globals.alliance);
        telemetry.addLine("x:" + Globals.startPose.getX());
        telemetry.addLine("y:" + Globals.startPose.getY());
        telemetry.addLine("heading:" + Math.toDegrees(Globals.startPose.getHeading()));


        //Check it gets the right cords for pedro then change
        //Check if pedro will work from different locations
        //Check pedro driving
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.update();
        }
    }
}