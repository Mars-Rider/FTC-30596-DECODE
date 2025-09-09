package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    public HardwareMap map;
    public final Telemetry telemetry;

    public DcMotor LRL, LFB, RRL, RFB;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.map = hardwareMap;


        LRL = hardwareMap.get(DcMotor.class, "LRL");//2 C
        LFB = hardwareMap.get(DcMotor.class, "LFB");//0 C
        RRL = hardwareMap.get(DcMotor.class, "RRL");//3 C
        RFB = hardwareMap.get(DcMotor.class, "RFB");//1 C

        LRL.setDirection(DcMotor.Direction.FORWARD);
        LFB.setDirection(DcMotor.Direction.FORWARD);
        RRL.setDirection(DcMotor.Direction.REVERSE);
        RFB.setDirection(DcMotor.Direction.REVERSE);

        RRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LFB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RRL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LRL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void driveWithControllers(double forward, double strafe, double turn) {
        double avg = forward+strafe+turn;

        //double left = (forward*strafe*(Math.sin(Math.toRadians(225))*turn))/avg;

        RRL.setPower(strafe);
        RFB.setPower(forward);
        LRL.setPower(strafe);
        LFB.setPower(strafe);

    }

}
