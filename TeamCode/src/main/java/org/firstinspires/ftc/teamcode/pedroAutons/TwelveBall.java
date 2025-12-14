package org.firstinspires.ftc.teamcode.pedroAutons; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Globals;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "12 Ball", group = "Autons")
public class TwelveBall extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    Robot robot;
    private Pose startPose = new Pose(28.5, 10, Math.toRadians(-90)); // Backup Start Pose of our robot.
    private Pose scorePose = new Pose(60, 85, Math.toRadians(-45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private Pose set1 = new Pose(30, 60, Math.toRadians(-180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private Pose set2 = new Pose(set1.getX(), 60, Math.toRadians(-180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private Pose set3 = new Pose(set1.getX(), 60, Math.toRadians(-180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private Pose grab1 = new Pose(30, 60, Math.toRadians(-180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private Pose grab2 = new Pose(grab1.getX(), 130, Math.toRadians(-180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private Pose grab3 = new Pose(grab1.getX(), 135, Math.toRadians(-180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private Pose parkPose = new Pose(40, 60, Math.toRadians(-180)); // Highest (First Set) of Artifacts from the Spike Mark.


    private Path scorePreload;
    private PathChain setRow1, getRow1, resetRow1, scoreRow1, setRow2, getRow2, resetRow2, scoreRow2, setRow3, getRow3, resetRow3,scoreRow3, park;
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        setRow1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, set1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), set1.getHeading())
                .build();

        getRow1 = follower.pathBuilder()
                .addPath(new BezierLine(set1, grab1))
                .setLinearHeadingInterpolation(set1.getHeading(), grab1.getHeading())
                .build();

        resetRow1 = follower.pathBuilder()
                .addPath(new BezierLine(grab1, set1))
                .setLinearHeadingInterpolation(grab1.getHeading(), set1.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreRow1 = follower.pathBuilder()
                .addPath(new BezierLine(set1, scorePose))
                .setLinearHeadingInterpolation(set1.getHeading(), scorePose.getHeading())
                .build();

        setRow2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, set2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), set2.getHeading())
                .build();

        getRow2 = follower.pathBuilder()
                .addPath(new BezierLine(set2, grab2))
                .setLinearHeadingInterpolation(set2.getHeading(), grab2.getHeading())
                .build();

        resetRow2 = follower.pathBuilder()
                .addPath(new BezierLine(grab2, set2))
                .setLinearHeadingInterpolation(grab2.getHeading(), set2.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreRow2 = follower.pathBuilder()
                .addPath(new BezierLine(set2, scorePose))
                .setLinearHeadingInterpolation(set2.getHeading(), scorePose.getHeading())
                .build();

        setRow3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, set3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), set3.getHeading())
                .build();

        getRow3 = follower.pathBuilder()
                .addPath(new BezierLine(set3, grab3))
                .setLinearHeadingInterpolation(set3.getHeading(), grab3.getHeading())
                .build();

        resetRow3 = follower.pathBuilder()
                .addPath(new BezierLine(grab3, set3))
                .setLinearHeadingInterpolation(grab3.getHeading(), set3.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreRow3 = follower.pathBuilder()
                .addPath(new BezierLine(set3, scorePose))
                .setLinearHeadingInterpolation(set3.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);

                setPathState(1);
                break;

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {

                    robot.outtakeByCode(Globals.code);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(setRow1, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    //run Intake
                    robot.autoSorting(true);
                    robot.intakePower(true);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(getRow1);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(resetRow1, true);
                    setPathState(4);
                } else {
                    if (robot.color(robot.pColor[0]) != 0 || robot.color(robot.pColor[0]) != 1) {
                        robot.setOuttake(1, true);
                    } else {
                        robot.setOuttake(1, false);
                    }

                    if (robot.color(robot.gColor[0]) != 0 || robot.color(robot.gColor[0]) != 2) {
                        robot.setOuttake(2, true);
                    } else {
                        robot.setOuttake(2, false);
                    }
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    robot.setOuttake(2, false);
                    robot.setOuttake(2, false);

                    robot.autoSorting(false);
                    robot.intakePower(false);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scoreRow1);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    robot.outtakeByCode();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(setRow2, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    //Purple

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(getRow2);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(resetRow2);
                    setPathState(8);
                } else {
                    if (robot.color(robot.pColor[0]) != 0 || robot.color(robot.pColor[0]) != 1) {
                        robot.setOuttake(1, true);
                    } else {
                        robot.setOuttake(1, false);
                    }

                    if (robot.color(robot.gColor[0]) != 0 || robot.color(robot.gColor[0]) != 2) {
                        robot.setOuttake(2, true);
                    } else {
                        robot.setOuttake(2, false);
                    }
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    robot.setOuttake(2, false);
                    robot.setOuttake(2, false);

                    robot.autoSorting(false);
                    robot.intakePower(false);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scoreRow2, true);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    robot.outtakeByCode();

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(setRow3);
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    //Purple

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(getRow3);
                    setPathState(11);
                }
                break;
            case 11:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(resetRow3);
                    setPathState(12);
                } else {
                    if (robot.color(robot.pColor[0]) != 0 || robot.color(robot.pColor[0]) != 1) {
                        robot.setOuttake(1, true);
                    } else {
                        robot.setOuttake(1, false);
                    }

                    if (robot.color(robot.gColor[0]) != 0 || robot.color(robot.gColor[0]) != 2) {
                        robot.setOuttake(2, true);
                    } else {
                        robot.setOuttake(2, false);
                    }
                }
                break;
            case 12:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    robot.setOuttake(2, false);
                    robot.setOuttake(2, false);

                    robot.autoSorting(false);
                    robot.intakePower(false);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scoreRow3, true);
                    setPathState(13);
                }
                break;
            case 13:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    robot.outtakeByCode(Globals.code);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(park);
                    setPathState(14);
                }
                break;
            case 14:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }

                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading()-90);//Change the heading by idk (180 or 90)
        telemetry.update();

        Globals.startPose = follower.getPose();
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        robot = new Robot(hardwareMap, telemetry);
        Globals.opMode = true;

        robot.readFieldData(true);
        startPose = Globals.startPose;

        scorePose = new Pose(robot.changeAlliance(scorePose.getX()), scorePose.getY(), Globals.alliance == 1 ? Math.toRadians(-45) : Math.toRadians(-135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        parkPose = new Pose(robot.changeAlliance(parkPose.getX()), parkPose.getY(), Globals.alliance == 1 ? Math.toRadians(-180) : Math.toRadians(0));
        set1 = new Pose(robot.changeAlliance(set1.getX()), set1.getY(), parkPose.getHeading()); // Highest (First Set) of Artifacts from the Spike Mark.
        set2 = new Pose(set1.getX(), set2.getY(), parkPose.getHeading()); // Highest (First Set) of Artifacts from the Spike Mark.
        set3 = new Pose(set1.getX(), set3.getY(), parkPose.getHeading()); // Highest (First Set) of Artifacts from the Spike Mark.
        grab1 = new Pose(robot.changeAlliance(grab1.getX()), grab1.getY(), parkPose.getHeading()); // Highest (First Set) of Artifacts from the Spike Mark.
        grab2 = new Pose(grab1.getX(), grab2.getY(), parkPose.getHeading()); // Middle (Second Set) of Artifacts from the Spike Mark.
        grab3 = new Pose(grab1.getX(), grab2.getY(), parkPose.getHeading()); // Lowest (Third Set) of Artifacts from the Spike Mark.

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);

        robot.readFieldData(true);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}