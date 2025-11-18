package org.firstinspires.ftc.teamcode.pedroAutons;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Globals;
import org.firstinspires.ftc.teamcode.Robot.Robot;

import java.util.Arrays;

@Autonomous
public class IllTry extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.readFieldData();
    }

    @Override
    public void start() {
        telemetry.addLine(Arrays.toString(Globals.code));
        telemetry.update();
        robot.estimatePower();
        robot.flySpeed = .675;
        robot.outtakeByCode();
    }

    @Override
    public void loop() {

    }
}
