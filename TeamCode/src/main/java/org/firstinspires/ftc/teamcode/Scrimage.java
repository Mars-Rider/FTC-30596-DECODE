package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.CustomIMU;
import com.pedropathing.ftc.localization.RevHubIMU;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Robot.Globals;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Robot.Settings;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Scrimmage")
public class Scrimage extends LinearOpMode{

    private Limelight3A limelight;

    public static Follower follower;

    private CustomIMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        //Run at Initialization
        boolean lastTrigger = false;

        Robot robot = new Robot(hardwareMap, telemetry);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        double distanceFromGoal = 0;
        boolean blueGoal = true;
        Pose currentPose = new Pose(0, 0);
        double autoAimTX = 0;
        boolean flywheelEnabled = false;
        boolean justChangedFlywheel = false;
        boolean intakeToggled = false;
        boolean intakeReversed = false;
        boolean intakeJustChanged = false;

        imu = new RevHubIMU();
        imu.initialize(hardwareMap, "imu", new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));

        // robot.startDrivetrain();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose());

        follower.startTeleopDrive();
        follower.update();
        //robot.readFieldData();

        Robot.flySpeed = 67.5;

        //LEDChannel auxLEDs = robot.LEDs.addChannel("auxLEDs");

        waitForStart();
        limelight.start();
        //On Start
        if (Globals.code[0] == 0) {
            //robot.readFieldData();
        }

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            robot.update();
            telemetry.update();

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                    currentPose = new Pose(botpose.getPosition().x, botpose.getPosition().y);
                    autoAimTX = result.getTx();
                } else autoAimTX = 0;
            } else autoAimTX = 0;

            if (robot.overide(gamepad1.dpad_up, gamepad2.dpad_up)) blueGoal = true;
            else if (robot.overide(gamepad1.dpad_down, gamepad2.dpad_down)) blueGoal = false;

            //robot.drivetrain.drive(-robot.overide(gamepad1.left_stick_x,gamepad2.left_stick_x), -robot.overide(gamepad1.left_stick_y,gamepad2.left_stick_y), -robot.overide(gamepad1.right_stick_x,gamepad2.right_stick_x) + robot.overide(gamepad1.left_trigger, gamepad2.left_trigger) * autoAimTX * Configs.turnFeedforward);

            double driveAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            double driveMag = Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            double drivePower = robot.applyStaticFeedforward(Math.abs(Math.pow(driveMag, 3)) * Math.signum(driveMag), 0.05);

            follower.setTeleOpDrive(
                    drivePower*Math.sin(driveAngle),
                    drivePower*Math.cos(driveAngle),
                    robot.applyStaticFeedforward(0.8 * Math.abs(Math.pow(gamepad1.right_stick_x, 3)) * Math.signum(gamepad1.right_stick_x), 0.05),
                    true);
            follower.update();


            if (robot.overide(gamepad1.right_stick_button,gamepad2.right_stick_button)) {
                Pose curentPose = follower.getPose();
                follower.setPose(new Pose(curentPose.getX(), curentPose.getY(), 0));
                imu.resetYaw();
            }



            if (robot.overide(gamepad1.right_stick_button,gamepad2.right_stick_button)) {
                Pose curentPose = follower.getPose();
                follower.setPose(new Pose(curentPose.getX(), curentPose.getY(), 0));
            }

            // if (robot.getFlywheelRPM() < 100 || Math.abs(Configs.testOuttakeRPM - robot.getFlywheelRPM()) < 75) {}
            if(robot.overide(gamepad1.xWasPressed(),gamepad2.xWasPressed())){robot.outtake(1, true);}
            if(robot.overide(gamepad1.bWasPressed(),gamepad2.bWasPressed())){robot.outtake(2, true);}
            if(robot.overide(gamepad1.xWasReleased(),gamepad2.xWasReleased())){robot.outtake(1, false);}
            if(robot.overide(gamepad1.bWasReleased(),gamepad2.bWasReleased())){robot.outtake(2, false);}


            //if(robot.overide(gamepad1.xWasReleased(),gamepad2.xWasReleased())){robot.outtake(0);}
            //if(robot.overide(gamepad1.bWasReleased(),gamepad2.bWasReleased())){robot.outtake(0);}

            if(robot.overide(gamepad1.aWasPressed(),gamepad2.aWasPressed())){robot.flyPower();}
            //if(robot.overide(gamepad1.yWasPressed(),gamepad2.yWasPressed())){robot.intakePower();}

            if(robot.overide(gamepad1.dpad_left, gamepad2.dpad_left)){
                robot.sort(2);
            } else if (robot.overide(gamepad1.dpad_right, gamepad2.dpad_right)){
                robot.sort(1);
            } else if(robot.overide(gamepad1.dpadDownWasPressed(), gamepad2.dpadDownWasPressed())){
                robot.autoSorting();
            } else {
                robot.sort();
            }


            if (robot.overide(gamepad1.y, gamepad2.y)) {
                if (!intakeJustChanged) intakeToggled = !intakeToggled;
                intakeJustChanged = true;
            } else if (robot.overide(gamepad1.left_bumper, gamepad2.left_bumper)) {
                if (!intakeJustChanged) intakeReversed = !intakeReversed;
                intakeJustChanged = true;
            } else intakeJustChanged = false;

            robot.intake.setPower(((intakeToggled)? 1:0) * ((intakeReversed)? -1:1));

            if(gamepad1.leftBumperWasPressed()){
                //robot.drivetrain.slowMode();
            }

            //if (robot.overide(gamepad1.left_trigger, gamepad2.left_trigger) > 0.1 && distanceFromGoal > 5) {
            //    robot.runFlywheelForDistance(distanceFromGoal);
            if (robot.overide(gamepad1.a, gamepad2.a)) {
                if (!justChangedFlywheel) flywheelEnabled = !flywheelEnabled;
                justChangedFlywheel = true;
            } else justChangedFlywheel = false;

            if (flywheelEnabled) robot.runFlywheel(Configs.testOuttakeRPM);
            else robot.runFlywheel(0);

            if (blueGoal) telemetry.addLine("Currently aiming at BLUE");
            else telemetry.addLine("Currently aiming at RED");

            if (currentPose.getX() == 0 && currentPose.getY() == 0) {
                telemetry.addLine("Waiting to see apriltag");
            } else {
                if (blueGoal) distanceFromGoal = Configs.BlueGoalTarget.distanceFrom(currentPose);
                else distanceFromGoal = Configs.RedGoalTarget.distanceFrom(currentPose);

                telemetry.addData("Current Distance from goal", distanceFromGoal);

                //telemetry.addLine("Fly Wheel Speed: "+ Robot.flySpeed);
                telemetry.addData("Target Flywheel RPM", robot.calculateFlywheelVel(distanceFromGoal));

            }

            telemetry.addData("Target Flywheel RPM", Configs.testOuttakeRPM);
            telemetry.addData("Actual Flywheel RPM", robot.getFlywheelRPM());
            telemetry.addData("Imu heading", Math.toDegrees(imu.getHeading()));
            telemetry.addData("Normalized heading", Math.toDegrees(MathFunctions.normalizeAngle(imu.getHeading())));
            telemetry.addLine("Sort On: "+ robot.sortOn);
            // telemetry.addLine("fly encoder: "+ robot.bFly.getVelocity(AngleUnit.RADIANS));
        }
    }
}