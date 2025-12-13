package org.firstinspires.ftc.teamcode.pedroAutons; // make sure this aligns with class location

import static android.os.SystemClock.sleep;

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

@Autonomous(name = "6 Ball", group = "Autons")
public class SixBall extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    Robot robot;
    private Pose startPose = new Pose(28.5, 10, Math.toRadians(-90)); // Backup Start Pose of our robot.
    private Pose scorePose = new Pose(60, 85, Math.toRadians(-45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private Pose parkPose = new Pose(30, 60, Math.toRadians(-180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private Pose humanPose = new Pose(43, 130, Math.toRadians(-180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private Pose playerPose = new Pose(49, 135, Math.toRadians(-180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private Path scorePreload;
    private PathChain findBall, setBall, grabBall, scoreBall, park;
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        findBall = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, humanPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), humanPose.getHeading())
                .build();

        grabBall = follower.pathBuilder()
                .addPath(new BezierLine(humanPose, playerPose))
                .setLinearHeadingInterpolation(humanPose.getHeading(), playerPose.getHeading())
                .build();

        setBall = follower.pathBuilder()
                .addPath(new BezierLine(playerPose, humanPose))
                .setLinearHeadingInterpolation(playerPose.getHeading(), humanPose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreBall = follower.pathBuilder()
                .addPath(new BezierLine(humanPose, scorePose))
                .setLinearHeadingInterpolation(humanPose.getHeading(), scorePose.getHeading())
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
                    follower.followPath(findBall, true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    //run Intake
                    robot.autoSorting(true);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabBall);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    //run the outtake for a bit - Purple
                    robot.outtake(1);
                    sleep(1000);
                    robot.outtake(1);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(setBall, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabBall);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    //run the outtake for a bit - Purple
                    robot.outtake(1);
                    sleep(1000);
                    robot.outtake(1);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(setBall, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    //Purple

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabBall);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    robot.outtake(2);
                    sleep(1000);
                    robot.outtake(2);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(setBall);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scoreBall, true);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (!follower.isBusy()) {
                    robot.outtakeByCode(Globals.code);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(park);
                    setPathState(10);
                }
                break;
            case 10:
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

        //Globals.startPose = follower.getPose();
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

        scorePose = new Pose(changeAlliance(40), 85, Globals.alliance == 1 ? Math.toRadians(-45) : Math.toRadians(-135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        parkPose = new Pose(changeAlliance(40), 30, Globals.alliance == 1 ? Math.toRadians(-180) : Math.toRadians(0));
        humanPose = new Pose(changeAlliance(40), 30, Globals.alliance == 1 ? Math.toRadians(-180) : Math.toRadians(0));
        playerPose = new Pose(changeAlliance(12), 30, Globals.alliance == 1 ? Math.toRadians(-180) : Math.toRadians(0));

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
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }

    public double changeAlliance(double input){
        if(Globals.alliance == 2){
            return (140-input);
        } else {
            return input;
        }
    }

}