package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Drive Test - All Forward", group = "Test")
public class DriveTestAllForward extends LinearOpMode {

    DcMotor LF, RF, LB, RB;

    @Override
    public void runOpMode() {

        LF = hardwareMap.get(DcMotor.class, "LFB");
        RF = hardwareMap.get(DcMotor.class, "RFB");
        LB = hardwareMap.get(DcMotor.class, "LRL");
        RB = hardwareMap.get(DcMotor.class, "RRL");

        // TEMPORARILY set all motors to FORWARD
        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            LF.setPower(0.5);
            RF.setPower(0.5);
            LB.setPower(0.5);
            RB.setPower(0.5);
        }
    }
}
