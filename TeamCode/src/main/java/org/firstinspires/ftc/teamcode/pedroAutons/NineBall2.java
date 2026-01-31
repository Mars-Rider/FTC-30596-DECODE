package org.firstinspires.ftc.teamcode.pedroAutons; // make sure this aligns with class location

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot.Globals;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "9 Ball In Score Zone", group = "Autons")
public class NineBall2 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    Robot robot;
    private Pose startPose = new Pose(28.5, 10, Math.toRadians(-90)); // Backup Start Pose of our robot.
    private Pose scorePose = new Pose(40, 135, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private Pose parkPose = new Pose(30, 60, Math.toRadians(-180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private Pose set1 = new Pose(50, 50, Math.toRadians(-180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private Pose grab1 = new Pose(10, 50, Math.toRadians(-180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private Pose reset1 = new Pose(50, 50, Math.toRadians(-90)); // Highest (First Set) of Artifacts from the Spike Mark.
    private Pose set2 = new Pose(50, 75, Math.toRadians(-180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private Pose grab2 = new Pose(20, 75, Math.toRadians(-180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private Pose reset2 = new Pose(50, 75, Math.toRadians(-90)); // Highest (First Set) of Artifacts from the Spike Mark.
    private Pose gate = new Pose(35, 65, Math.toRadians(-180)); // Highest (First Set) of Artifacts from the Spike Mark.

    //private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(-180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    //private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(-180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private Path scorePreload;
    private PathChain setRow1, getRow1, resetRow1, scoreRow1,setRow2, getRow2, resetRow2,scoreRow2,park;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

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
                .addPath(new BezierLine(grab1, reset1))
                .setLinearHeadingInterpolation(grab1.getHeading(), reset1.getHeading())
                .build();

        scoreRow1 = follower.pathBuilder()
                .addPath(new BezierLine(reset1, scorePose))
                .setLinearHeadingInterpolation(reset1.getHeading(), scorePose.getHeading())
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
                .addPath(new BezierLine(grab2, reset2))
                .setLinearHeadingInterpolation(grab2.getHeading(), reset2.getHeading())
                .build();

        scoreRow2 = follower.pathBuilder()
                .addPath(new BezierLine(reset2, scorePose))
                .setLinearHeadingInterpolation(reset2.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, gate))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gate.getHeading())
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
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    robot.intakePower(true);
                    robot.sort(2);
                    robot.outtake(2, true);
                    robot.outtake(2, false);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(getRow1, true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (2 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    robot.outtake(2, false);
                    robot.outtake(1, false);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(resetRow1, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (2 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    robot.intakePower(false);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scoreRow1, true);
                    robot.flyPower(true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (2 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    robot.outtake(2, true);
                    robot.outtake(1, true);
                    sleep(3000);
                    robot.outtake(2, false);
                    robot.outtake(1, false);
                    robot.flyPower(false);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(setRow2, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (2 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    robot.intakePower(true);
                    robot.sort(2);
                    robot.outtake(2, true);
                    robot.outtake(1, true);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(getRow2, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (2 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    robot.outtake(2, false);
                    robot.outtake(1, false);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(resetRow2, true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (2 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    robot.intakePower(false);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(scoreRow2, true);
                    robot.flyPower(true);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (2 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    robot.outtake(2, true);
                    robot.outtake(1, true);
                    sleep(3000);
                    robot.outtake(2, false);
                    robot.outtake(1, false);
                    robot.flyPower(false);

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(park, true);
                    setPathState(10);
                }
                break;
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }

                Globals.finishedAuto = true;

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
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));//Change the heading by idk (180 or 90)
        telemetry.update();

        Globals.startPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading());
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
        Globals.finishedAuto = false;

        scorePose = new Pose(robot.changeAlliance(scorePose.getX()), scorePose.getY(), Globals.alliance == 1 ? Math.toRadians(0) : Math.toRadians(-180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
        parkPose = new Pose(robot.changeAlliance(parkPose.getX()), parkPose.getY(), Globals.alliance == 1 ? Math.toRadians(-180) : Math.toRadians(0));
        set1 = new Pose(robot.changeAlliance(set1.getX()), set1.getY(), Globals.alliance == 1 ? Math.toRadians(-180) : Math.toRadians(0));
        grab1 = new Pose(robot.changeAlliance(grab1.getX()), grab1.getY(), Globals.alliance == 1 ? Math.toRadians(-180) : Math.toRadians(0));
        reset1 = new Pose(robot.changeAlliance(reset1.getX()), reset1.getY(), Globals.alliance == 1 ? Math.toRadians(-90) : Math.toRadians(90));
        set2 = new Pose(robot.changeAlliance(set2.getX()), set2.getY(), Globals.alliance == 1 ? Math.toRadians(-180) : Math.toRadians(0));
        grab2 = new Pose(robot.changeAlliance(grab2.getX()), grab2.getY(), Globals.alliance == 1 ? Math.toRadians(-180) : Math.toRadians(0));
        reset2 = new Pose(robot.changeAlliance(reset2.getX()), reset2.getY(), Globals.alliance == 1 ? Math.toRadians(-90) : Math.toRadians(90));
        gate = new Pose(robot.changeAlliance(gate.getX()), gate.getY(), Globals.alliance == 1 ? Math.toRadians(-180) : Math.toRadians(0));


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