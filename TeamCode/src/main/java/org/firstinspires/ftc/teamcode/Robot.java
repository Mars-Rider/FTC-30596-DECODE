package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.pedropathing.ftc.localization.RevHubIMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Robot {
    public HardwareMap map;
    public final Telemetry telemetry;

    public DcMotor LRL, LFB, RRL, RFB;
    public DcMotor pFly, gFly, intake;
    public Servo sort;

    private double sortMid = 0.5;
    private double sortInc = 0.2;

    public double driveSpeed = 0.5; //Default speed of drivetrain
    public double driveSpeedSlow = 0.1; //Speed of drivetrain when in slow mode
    public boolean facingGoal = false; //If true, robot is facing the goal at all times and controls turn into global x and y, False is no more facing goal and it uses local driving

    static public int alliance = 0; //0 = N/A, 1 = Blue, 2 = Red
    private int[] codeIDs = {21 , 22 , 23}; //Put the ids for each code here
    private int[][] codes = {{2,1,1},{1,2,1},{1,1,2}};
    static public int[] code = {4,4,4}; //0 = No ball, 1 = Purple, 2 = Green
    public int[] loaded = {0, 0, 0}; //Order of balls that are loaded - 0 = No ball, 1 = Purple, 2 = Green

    private boolean fly = false; //True = on
    private double dFlySpeed = 0.5; //Default speed of flywheel
    public double flySpeed = 0.5; //Speed of flywheel
    public double flySpeedIncre = 0.1; //Default increase/dececrease of the speed of flywheel
    public boolean incremented = false;
    private Servo[] pRoll; //Purple Ball Rollers
    private Servo[] gRoll; //Green Ball Rollers
    private double rollSpeed = 0.75; //Speed of the rollers
    private boolean intakeOn = false; //True = on
    private double intakeSpeed = 0.5; //Speed of intake

    private HuskyLens huskyLens;
    private Limelight3A limelight;
    private IMU imu;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.map = hardwareMap;

        if(code[1] != 0 && code[1] != 1 && code[1] != 2) {
            code[0] = 0;
            code[1] = 0;
            code[2] = 0;
        }

        //Drivetrain
        LRL = hardwareMap.get(DcMotor.class, "LRL");//2 C (Port 2, Control Hub Motors)
        LFB = hardwareMap.get(DcMotor.class, "LFB");//0 C
        RRL = hardwareMap.get(DcMotor.class, "RRL");//3 C
        RFB = hardwareMap.get(DcMotor.class, "RFB");//1 C

        //Flywheels
        //pFly = hardwareMap.get(DcMotor.class, "pFly");//0 E (Port 0, Expansion Hub Motors)
        //gFly = hardwareMap.get(DcMotor.class, "gFly");//1 E

        //Rollers
        //pRoll[0] = hardwareMap.get(Servo.class, "pR1");//1 ES
        //pRoll[1] = hardwareMap.get(Servo.class, "pR2");//2 ES
        //gRoll[0] = hardwareMap.get(Servo.class, "gR1");//4 ES
        //gRoll[1] = hardwareMap.get(Servo.class, "gR2");//5 ES

        //Intake
        //intake = hardwareMap.get(DcMotor.class, "intake");//3 E
        sort = hardwareMap.get(Servo.class, "sort");//3 ES (Port 3, Expansion Hub Servo Slots)
        sort.setDirection(Servo.Direction.FORWARD);

        LRL.setDirection(DcMotor.Direction.FORWARD);
        LFB.setDirection(DcMotor.Direction.FORWARD);
        RRL.setDirection(DcMotor.Direction.REVERSE);
        RFB.setDirection(DcMotor.Direction.REVERSE);
        //intake.setDirection(DcMotor.Direction.FORWARD);
//        pFly.setDirection(DcMotor.Direction.FORWARD);
//        gFly.setDirection(DcMotor.Direction.FORWARD);

        RRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LFB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        pFly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        gFly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RRL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LRL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       //intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        pFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        gFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Huksy Lens
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        //Limelights
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(8);// See limelight piplines (Whimsical)
//        limelight.start();

        //IMU - Localizer from LimeLight (Gets bot pos from the april tags - so tough)
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
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
    public boolean triggerAsButton(double news){
        double analogThreshold = 0.25;

        return Math.abs(news) > analogThreshold;
    }

    /*public void driveWithControllers(double forward, double strafe, double turn, boolean scale) {

        double A = forward - turn;
        double B = forward + turn;
        double RightPower = Math.hypot(strafe, A);
        double LeftPower = Math.hypot(strafe, B);
        double max = Math.max(1, Math.max(RightPower, LeftPower));

        double scalar;
        if(scale){scalar=driveSpeed;}else{scalar=driveSpeedSlow;}

        controlRightPod(Math.atan2(strafe, A), (RightPower/max) * scalar);
        controlLeftPod(Math.atan2(strafe, B), (LeftPower/max) * scalar);
    }*/ //Pedro Teleop instead - just put all drive in the pedro

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

//    //Flywheels
//    public void flyPower() {
//        if(!fly){
//            pFly.setPower(flySpeed);
//            gFly.setPower(flySpeed);
//            fly = true;
//        } else {
//            pFly.setPower(0);
//            gFly.setPower(0);
//            fly = false;
//        }
//    } //Turn on and off power of flywheel
//
//    public void flyPower(boolean manual) { //True is on, false is off
//        if(fly != manual){ //Only go forward if manual isn't the same as fly
//            if(manual){//True is on
//                pFly.setPower(flySpeed);
//                gFly.setPower(flySpeed);
//                fly = true;
//            } else {
//                pFly.setPower(0);
//                gFly.setPower(0);
//                fly = false;
//            }
//        }
//    } //Turn on and off power of flywheel based off of what the input is
//
//    public void outtake(int color) {
//        if(!fly){
//            flyPower(true);
//            if(color == 1) { //Purple
//                for (Servo servo : pRoll) {
//                    servo.setPosition(rollSpeed);
//                }
//            } else if (color == 2) { // Green
//                for (Servo servo : gRoll) {
//                    servo.setPosition(rollSpeed);
//                }
//            }
//        }
//    } //Turn on power if needed and spin rollers based on the color
//
//    public void outtakeByCode() {
//        int totLoad = 0;
//        int totCode = 0;
//        for (int l = 0; l < loaded.length; l++) {
//            if(loaded[l] != 0) {totLoad += loaded[l];}
//            if(code[l] != 0) { totCode += code[l];}
//        }
//
//        if(totLoad == totCode){
//            for (int color:code) {
//                outtake(color);
//
//                sleep(500); //Wait 500ms then do the next one
//            }
//        }
//
//    } //Automatically shoots by the code

    //Intake
    public void intakePower() {
        if(!intakeOn){
            //intake.setPower(intakeSpeed);
            intakeOn = true;
        } else {
            //intake.setPower(0);
            intakeOn = false;
        }
    } //Turn on and off power of intake

    public void intakePower(boolean manual) { //True is on, false is off
        if(intakeOn != manual){ //Only go forward if manual isn't the same as fly
            if(manual){//True is on
                //intake.setPower(intakeSpeed);
                intakeOn = true;
            } else {
                //intake.setPower(0);
                intakeOn = false;
            }
        }
    } //Turn on and off power of intake based off of what the input is

    public int closestColor() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);
        int closestY = 0;
        int closestHeight = 0;
        int closestColor = 0;
        for (HuskyLens.Block b : blocks) {
            if(b.y + (b.height/2) > closestY){ //Find color that is closest to the bottom ((0,0) is top left)
                closestY = b.y + (b.height/2);
                closestHeight = b.height;
                closestColor = b.id;
                //When setting the colors, id for purple is 1 and id for green id 2
            } else if (b.y + (b.height/2) == closestY){ //Find color that is closest to the bottom and smallest height (closest)
                if(b.height < closestHeight){
                    closestHeight = b.height;
                    closestColor = b.id;
                }
            }

            /*
             * Here inside the FOR loop, you could save or evaluate specific info for the currently recognized Bounding Box:
             * - blocks[i].width and blocks[i].height   (size of box, in pixels)
             * - blocks[i].left and blocks[i].top       (edges of box)
             * - blocks[i].x and blocks[i].y            (center location)
             * - blocks[i].id                           (Color ID)
             *
             * These values have Java type int (integer).
             */
        }

        return closestColor; //Return the color that comes
    } //Finds the closest color that is infront of the robot (Husky lens)

    public void sort() {
        intakePower(true);

        int color = closestColor();

        if(color == 1){
            sort.setPosition(sortMid+sortInc);
            return;
        }else if (color == 2){
            sort.setPosition(sortMid-sortInc);
            return;
        }
    }

    public void sort(int color) {
        intakePower(true);

        if(color == 1){
            sort.setPosition(sortMid+sortInc);
            return;
        }else if (color == 2){
            sort.setPosition(sortMid-sortInc);
            return;
        }
    } //0 = No ball, 1 = Purple, 2 = Green

    //Limelight
    public void readFieldData() {
        int codeID = 0;
        int allianceID = 0;

        //Get data
//        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//        limelight.updateRobotOrientation(orientation.getYaw());
//        LLResult llResult = limelight.getLatestResult();
//        if(llResult != null && llResult.isValid()){
//            codeID = llResult.getFiducialResults().get(0).getFiducialId();
//            allianceID = llResult.getFiducialResults().get(1).getFiducialId();//Get second april tag, should be alliance tag
//        }
//
//        if(allianceID == 20){alliance = 1;} else if (allianceID== 24){alliance = 2;}
//
//        for (int i = 0; i < codeIDs.length; i++) {
//            if (codeIDs[i] == codeID) {
//                codeID = i;
//                break;// return index when found
//            }
//        }

        code = codes[codeID];
    } //Sets the code
    public void faceGoal() {}//Track april tag
    public void estimatePower() {} //Find distance and get needed power

    public void start(){
//        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//        limelight.updateRobotOrientation(orientation.getYaw());
//        LLResult llResult = limelight.getLatestResult();
//        if(llResult != null && llResult.isValid()){
//            Pose3D botPose = llResult.getBotpose_MT2(); //Get position in relation to april tag (i think 90 is straight on)
//            telemetry.addData("Tx", llResult.getTx());
//            telemetry.addData("Ty", llResult.getTy());
//            telemetry.addData("Ta", llResult.getTa());
//        }
//
//        readFieldData();
    } //Put things to do in the loop here

    public void update(){
        incremented = false;

//        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//        limelight.updateRobotOrientation(orientation.getYaw());
//        LLResult llResult = limelight.getLatestResult();
//        if(llResult != null && llResult.isValid()){
//            Pose3D botPose = llResult.getBotpose_MT2(); //Get position in relation to april tag (i think 90 is straight on)
//            telemetry.addData("Tx", llResult.getTx());
//            telemetry.addData("Ty", llResult.getTy());
//            telemetry.addData("Ta", llResult.getTa());
//        }
//
//        if(facingGoal){
//            faceGoal();
//        }

        sort();
    } //Put things to do in the loop here
}
