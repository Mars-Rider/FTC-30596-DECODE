package org.firstinspires.ftc.teamcode.Robot;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.Robot.Robot;

import java.util.function.Supplier;

public class Drivetrain {
    public HardwareMap map;
    public Telemetry telemetry;

    public DcMotor LRL, LFB, RRL, RFB;
    public static double driveSpeed = 1; //Default speed of drivetrain
    public static double driveSpeedSlow = 0.5; //Speed of drivetrain when in slow mode
    public static boolean slowMode = false;

    public static double[] auxInputs = {0, 0, 0};

    private Follower follower;
    public Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    public boolean pedroDriving = false;

    private void startDrivetrain(HardwareMap hardwareMap, Telemetry telemetry, boolean pedro){
        if(pedro){
            pedroDriving = true;
            follower = Constants.createFollower(map);
            startingPose = Globals.startPose;
            telemetry.addLine("Starting Heading: " + startingPose.getHeading());
            /*if (Globals.finishedAuto == true) {
                startingPose = startingPose.setHeading(-180);
            }*/
            follower.setStartingPose(startingPose == null ? new Pose(Robot.changeAlliance(28),10,Math.toRadians(-90)) : startingPose);
            follower.update();
            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        } else {
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
    }

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry, boolean pedro) {
        this.telemetry = telemetry;
        this.map = hardwareMap;

        startDrivetrain(hardwareMap,telemetry,pedro);
    }

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.map = hardwareMap;

        startDrivetrain(hardwareMap,telemetry,false);
    }

    public void runTo(Pose pose){
        if(!pedroDriving){return;}

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, pose)))
                .setConstantHeadingInterpolation(follower.getHeading())
                .build();

        follower.followPath(pathChain.get());
        automatedDrive = true;
    }

    public void runTo(Pose pose, double endHeading){
        if(!pedroDriving){return;}

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, pose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, pose.getHeading(), 0.8))
                .build();

        follower.followPath(pathChain.get());
        automatedDrive = true;
    }

    public void runTo(double endHeading){
        if(!pedroDriving){return;}

        Pose pose = new Pose(follower.getPose().getX() + 0.1,follower.getPose().getY() + 0.1,endHeading);

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, pose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, pose.getHeading(), 0.8))
                .build();

        follower.followPath(pathChain.get());
        automatedDrive = true;
    }

    public Pose getPosition(){
        return follower.getPose();
    }

    public double getHeading(){
        return follower.getHeading();
    }

    public void slowMode(){
        slowMode(!slowMode);
    }

    public void slowMode(boolean manual){
        slowMode = manual;
    }

    public void update(){

        if(pedroDriving){
            follower.update();
            telemetryM.update();
            telemetryM.debug("position", follower.getPose());
            telemetryM.debug("velocity", follower.getVelocity());
            telemetryM.debug("automatedDrive", automatedDrive);

            if (automatedDrive && (!follower.isBusy())) {
                follower.startTeleopDrive(true);
                automatedDrive = false;
            }
        }
    }

    public void begin(){

        if(pedroDriving){
            follower.startTeleopDrive(true);
        }
    }

    public void SwerveDrive(double x, double y, double r) {

    }

    public void drive(double x, double y, double r){
        drive(x,y,r,true);
    }

    public void drive(double x, double y, double r, boolean robotCentric){
        x += auxInputs[0];
        y += auxInputs[1];
        r += auxInputs[2];

        if(pedroDriving){
            if (!automatedDrive) {
                follower.setTeleOpDrive(
                        -y  * (slowMode ? driveSpeedSlow : driveSpeed),
                        -x  * (slowMode ? driveSpeedSlow : driveSpeed),
                        -r  * (slowMode ? driveSpeedSlow : driveSpeed),
                        robotCentric // Robot Centric
                );
            } else if(Math.abs(x) > 0.05 || Math.abs(y) > 0.05 || Math.abs(r) > 0.05){
                automatedDrive = false;
                follower.startTeleopDrive(true);
            }
        } else {
            double[] wheelPowers = {0,0,0,0}; //Fr, fl, br, bl

            y *= 1;

            wheelPowers[0] = (y-x-r) * (slowMode ? driveSpeedSlow : driveSpeed);
            wheelPowers[1] = (y+x+r) * (slowMode ? driveSpeedSlow : driveSpeed);
            wheelPowers[2] = (y-x+r) * (slowMode ? driveSpeedSlow : driveSpeed);
            wheelPowers[3] = (y+x-r) * (slowMode ? driveSpeedSlow : driveSpeed);

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
}
