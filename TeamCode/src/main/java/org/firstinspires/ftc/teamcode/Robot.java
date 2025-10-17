package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    public HardwareMap map;
    public final Telemetry telemetry;

    public DcMotor LRL, LFB, RRL, RFB;
    public DcMotor pFly, gFly, intake;
    public Servo sort;

    public int driveSpeed = 0.5; //Default speed of drivetrain
    public int driveSpeedSlow = 0.1 //Speed of drivetrain when in slow mode

    public int code[3]; //0 = No ball, 1 = Purple, 2 = Green
    public int loaded[3] = [0, 0, 0]; //Order of balls that are loaded - 0 = No ball, 1 = Purple, 2 = Green

    public boolean fly = false; //True = on
    public int flySpeed = 0.5; //Default speed of flywheel
    public int flySpeedIncre = 0.1 //Default increase/dececrease of the speed of flywheel
    public Servo[3] pRoll; //Purple Ball Rollers
    public Servo[3] gRoll; //Green Ball Rollers

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.map = hardwareMap;

        code[1] == null ? code[3] = [0, 0, 0];

        //Drivetrain
        LRL = hardwareMap.get(DcMotor.class, "LRL");//2 C (Port 2, Control Hub Motors)
        LFB = hardwareMap.get(DcMotor.class, "LFB");//0 C
        RRL = hardwareMap.get(DcMotor.class, "RRL");//3 C
        RFB = hardwareMap.get(DcMotor.class, "RFB");//1 C

        //Flywheels
        pFly = hardwareMap.get(DcMotor.class, "pFly");//0 E (Port 0, Expansion Hub Motors)
        gFly = hardwareMap.get(DcMotor.class, "gFly");//1 E

        //Intake
        RFB = hardwareMap.get(DcMotor.class, "RFB");//3 E
        Grab = hardwareMap.get(Servo.class, "sort");//3 ES (Port 3, Expansion Hub Servo Slots)

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

        //Huksy Lens
        //Limelights
    }

    public double overide (double primary, double override){
        if(override == 0){
            return primary;
        } else {
            return override;
        }
    }
    public boolean overide (boolean primary, boolean override){
        if(override){
            return override;
        } else {
            return primary;
        }
    }

    public void driveWithControllers(double forward, double strafe, double turn, boolean scale) {

        double A = forward - turn;
        double B = forward + turn;
        double RightPower = Math.hypot(strafe, A);
        double LeftPower = Math.hypot(strafe, B);
        double max = Math.max(1, Math.max(RightPower, LeftPower));

        double scalar;
        if(scale){scalar=dDriveSpeed}else{scalar=driveSpeedSlow}

        controlRightPod(Math.atan2(strafe, A), (RightPower/max) * scalar);
        controlLeftPod(Math.atan2(strafe, B), (LeftPower/max) * scalar);
    }

    public void controlRightPod(double Angle, double Power) {
        double strafePower = Power * Math.cos(Angle);
        double forwardPower = Power * Math.sin(Angle);
        RRL.setPower(strafePower);
        RFB.setPower(forwardPower);
    }

    public void controlLeftPod(double Angle, double Power) {
        double strafePower = Power * Math.cos(Angle);
        double forwardPower = Power * Math.sin(Angle);
        LRL.setPower(strafePower);
        LFB.setPower(forwardPower);
    }

    //Flywheels
    public void power() {
        if(!fly){
            //Turn on power
            fly = true;
        } else {
            //Turn off power
            fly = false;
        }
    } //Turn on and off power of flywheel

    public void power(boolean manual) { //True is on, false is off
        if(fly != manual){ //Only go forward if manual isn't the same as fly
            if(manual){ //True is on
                //turn on power
                fly = false;
            } else {
                //Turn off power
                fly = false;
            }
        }
    } //Turn on and off power of flywheel based off of what the input is

    public void outtake(int color) {
        if(!fly){
            power();
            if(color == 1) { //Purple

            } else if (color == 2) { // Green

            }
        }
    } //Turn on power if needed and spin rollers based on the color
    public void outtakeByCode() {
        int totLoad = 0;
        int totCode = 0;
        for (int l = 0; l < loaded.length; l++) {
            loaded[i-1] ! = 0 ? totLoad+loaded[i-1];
            code[i-1] != 0 ? totCode+code[i-1]
        }

        if(totLoad == totCode){
            for (int i = 0; i < code.length; i++) {
                if(code[i-1] == 1) {//Purple
                    outtake(1);
                } else if (code[i-1] == 2) {//Green
                    outtake(2);
                }

                //delay somehow
            }
        }

    } //Automatically shoots by the code

    //Intake
    public void closestColor() {} //Finds the closest color that is infront of the robot (Husky lens)
    public void sort(int color) {} //0 = No ball, 1 = Purple, 2 = Green

    //Limelight
    public void readObleisk() {}
    public void faceGoal() {}
    public void estimatePower() {} //Find distance and get needed power
}
