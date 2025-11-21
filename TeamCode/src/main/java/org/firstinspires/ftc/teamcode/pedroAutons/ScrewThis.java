package org.firstinspires.ftc.teamcode.pedroAutons;


import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous
public class ScrewThis extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.readFieldData();
    }

    @Override
    public void start() {
        robot.drivetrain.drive(0,-1,0);
        sleep(2000);
        robot.drivetrain.drive(0,0,0);
    }

    @Override
    public void loop() {

    }
}
