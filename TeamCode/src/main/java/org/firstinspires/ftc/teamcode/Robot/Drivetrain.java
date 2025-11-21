package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain {
    public HardwareMap map;
    public final Telemetry telemetry;

    public DcMotor LRL, LFB, RRL, RFB;
    public static double driveSpeed = 0.5; //Default speed of drivetrain
    public static double driveSpeedSlow = 0.1; //Speed of drivetrain when in slow mode

    public static double[] auxInputs = {0, 0, 0};

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.map = hardwareMap;

        //Drivetrain
        LRL = hardwareMap.get(DcMotor.class, "LRL");//2 C (Port 2, Control Hub Motors)
        LFB = hardwareMap.get(DcMotor.class, "LFB");//0 C
        RRL = hardwareMap.get(DcMotor.class, "RRL");//3 C
        RFB = hardwareMap.get(DcMotor.class, "RFB");//1 C

        LRL.setDirection(DcMotor.Direction.REVERSE);
        LFB.setDirection(DcMotor.Direction.REVERSE);
        RRL.setDirection(DcMotor.Direction.FORWARD);
        RFB.setDirection(DcMotor.Direction.FORWARD);

        RRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LFB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RRL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LRL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void SwerveDrive(double x, double y, double r) {

    }

    public void drive(double x, double y, double r){
        double[] wheelPowers = {0,0,0,0}; //Fr, fl, br, bl

        x += auxInputs[0];
        y += auxInputs[1];
        r += auxInputs[2];

        y *= -1;

        wheelPowers[0] = y-x-r;
        wheelPowers[1] = y+x+r;
        wheelPowers[2] = y-x+r;
        wheelPowers[3] = y+x-r;

        double m = Math.max(1,Math.max(Math.abs(wheelPowers[0]),Math.max(Math.abs(wheelPowers[1]),Math.max(Math.abs(wheelPowers[2]),Math.abs(wheelPowers[3])))));
        wheelPowers[0] /= m;
        wheelPowers[1] /= m;
        wheelPowers[2] /= m;
        wheelPowers[3] /= m;

        LFB.setPower(wheelPowers[1]);
        LRL.setPower(wheelPowers[2]);
        RFB.setPower(wheelPowers[0]);
        RRL.setPower(wheelPowers[3]);
    }
}
