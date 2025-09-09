package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOpMode extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        //Run at Initialization
        Robot robot = new Robot(hardwareMap, telemetry);


        waitForStart();
        //On Start


        if (isStopRequested()) return;

        while (opModeIsActive()) {

            robot.driveWithControllers(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }
}