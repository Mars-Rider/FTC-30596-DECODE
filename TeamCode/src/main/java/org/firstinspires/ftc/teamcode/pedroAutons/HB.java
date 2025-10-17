package org.firstinspires.ftc.teamcode.pedroAutons;

import static org.firstinspires.ftc.teamcode.InvKinematics.InvArm.Constants.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.InvKinematics.InvArm.InvArm;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "High Basket")
public class HB extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    follower.setDrivetrain

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 16 by 15 inches
     * Lets assume the Robot is facing the opposite side and we want to score in the bucket */

    //Poses
    private final Pose startPose = new Pose(8.1, 96, Math.toRadians(0));
    private final Pose forwardPose = new Pose(18, 96, Math.toRadians(0));
    private final Pose scorePose = new Pose(7, 136.5, Math.toRadians(135));
    /*
    private final Pose scorePose1 = new Pose(9, 137, Math.toRadians(135));
    private final Pose scorePose2 = new Pose(11, 139, Math.toRadians(135));

    INIALIZA5TION:
    line motor up wioth left moxt line
    wheels 1 pointer finger from the wall
    */
    private final Pose scorePose1 = new Pose(9.5, 138, Math.toRadians(135));
    private final Pose wait3Pose =  new Pose(9.5, 135.5, Math.toRadians(135));
    private final Pose scorePose2 = new Pose(10.5, 138.5, Math.toRadians(135));
    private final Pose pickUp1Pose = new Pose(29.25, 122.5, Math.toRadians(0));
    private final Pose wait1Pose = new Pose(29.5, 122.25, Math.toRadians(0));
    private final Pose pickUp2Pose = new Pose(32.75, 134.0, Math.toRadians(0));
    private final Pose wait2Pose = new Pose(32.5, 133.75, Math.toRadians(0));
    private final Pose rePositionPose = new Pose(28.5, 124, Math.toRadians(135));
    private final Pose endPose = new Pose(24, 120, Math.toRadians(135));

    private Path forward, endPos;
    private PathChain scorePreload, pickup1, score1, pickup2, score2, wait1, wait2, wait1a, wait2a, rePosition1, rePosition, wait3;
    public void buildPaths() {
        forward = new Path(new BezierLine(new Point(startPose), new Point(forwardPose)));
        forward.setLinearHeadingInterpolation(startPose.getHeading(), forwardPose.getHeading());

        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(forwardPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(forwardPose.getHeading(), scorePose.getHeading())
                .build();

        pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickUp1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickUp1Pose.getHeading())
                .build();

        wait1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUp1Pose), new Point(wait1Pose)))
                .setLinearHeadingInterpolation(pickUp1Pose.getHeading(), wait1Pose.getHeading())
                .build();

        wait1a = follower.pathBuilder()
                .addPath(new BezierLine(new Point(wait1Pose), new Point(pickUp1Pose)))
                .setLinearHeadingInterpolation(wait1Pose.getHeading(), pickUp1Pose.getHeading())
                .build();

        rePosition1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUp1Pose), new Point(rePositionPose)))
                .setLinearHeadingInterpolation(pickUp1Pose.getHeading(), rePositionPose.getHeading())
                .build();

        score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(rePositionPose), new Point(scorePose1)))
                .setLinearHeadingInterpolation(rePositionPose.getHeading(), scorePose1.getHeading())
                .build();

        wait3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose1), new Point(wait3Pose)))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), wait3Pose.getHeading())
                .build();

        pickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose1), new Point(pickUp2Pose)))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), pickUp2Pose.getHeading())
                .build();

        wait2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUp2Pose), new Point(wait2Pose)))
                .setLinearHeadingInterpolation(pickUp2Pose.getHeading(), wait2Pose.getHeading())
                .build();

        wait2a = follower.pathBuilder()
                .addPath(new BezierLine(new Point(wait2Pose), new Point(pickUp2Pose)))
                .setLinearHeadingInterpolation(wait2Pose.getHeading(), pickUp2Pose.getHeading())
                .build();

        rePosition = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUp2Pose), new Point(rePositionPose)))
                .setLinearHeadingInterpolation(pickUp2Pose.getHeading(), rePositionPose.getHeading())
                .build();

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(rePositionPose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(rePositionPose.getHeading(), scorePose2.getHeading())
                .build();

        endPos = new Path(new BezierLine(new Point(scorePose), new Point(endPose)));
        endPos.setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading());
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.setPosition(startingCoords);
                follower.followPath(forward);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    robot.setPosition(highBasket);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePreload,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    robot.setGrabber(true);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(pickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    robot.setPosition(infrontAuto);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(wait1,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    robot.setGrabber(false);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(wait1a,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    robot.setPosition(highBasket);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(rePosition1,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    robot.setPosition(highBasket);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score1,true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    robot.setGrabber(true);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(wait3,true);
                    setPathState(8);
                }
                //robot.setPosition(robot.topBasket);
                break;

            case 8:
                if(!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(pickup2,true);
                    setPathState(9);
                }
                    //robot.setPosition(robot.topBasket);
                break;
            case 9:
                if(!follower.isBusy()) {
                    robot.setPosition(new InvArm.armCoords(infront.x, infront.y-10,infront.wrist));

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(wait2,true);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    robot.setGrabber(false);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(wait2a,true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    robot.setPosition(highBasket);//Move wrist straight down so that it can grab it.\

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(rePosition,true);
                    //robot.setPosition(robot.topBasket);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    //robot.setPosition(robot.topBasket);//Move wrist straight down so that it can grab it.\

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(score2,true);
                    //robot.setPosition(robot.topBasket);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    robot.setGrabber(true);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(endPos,true);
                    setPathState(14);
                }
                break;
            case 14:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    Robot robot;

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        robot = new Robot(hardwareMap,telemetry,false);
        robot.resetEncoders();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
