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

@Autonomous
public class ScrewThis extends OpMode {
    Robot robot = new Robot(hardwareMap,telemetry);

    @Override
    public void init() {
        robot.readFieldData();
    }

    @Override
    public void start() {
        robot.drive(0,1,0);
        sleep(2000);
        robot.drive(0,0,0);
    }

    @Override
    public void loop() {

    }
}
