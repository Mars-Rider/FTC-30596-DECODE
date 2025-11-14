package org.firstinspires.ftc.teamcode.pedroAutons;


import static android.os.SystemClock.sleep;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Globals;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

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
