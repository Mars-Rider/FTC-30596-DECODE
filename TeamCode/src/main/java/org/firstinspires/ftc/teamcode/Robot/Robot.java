package org.firstinspires.ftc.teamcode.Robot;

import static android.os.SystemClock.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Swerve.Swerve;


@Configurable
public class Robot {
    public HardwareMap map;
    public final Telemetry telemetry;

    public Drivetrain drivetrain;

    public DcMotorEx bFly, tFly;
    public DcMotor intake;
    public Servo sort;

    public boolean opMode = false; //True is auton

    private final double sortMid = 0.5;
    public static double sortInc = 0.2;
    public boolean sortOn = false;//True is sorting

    public boolean facingGoal = false; //If true, robot is facing the goal at all times and controls turn into global x and y, False is no more facing goal and it uses local driving

    //static public int alliance = 0; //0 = N/A, 1 = Blue, 2 = Red
    private int[] codeIDs = {21 , 22 , 23}; //Put the ids for each code here
    private int[][] codes = {{2,1,1},{1,2,1},{1,1,2}};
    //static public int[] code = {4,4,4}; //0 = No ball, 1 = Purple, 2 = Green
    // public int[] loaded = {0, 0, 0}; //Order of balls that are loaded - 0 = No ball, 1 = Purple, 2 = Green

    private boolean fly = false; //True = on

    public double flySpeed = 200; //Speed of flywheel
    public double flySpeedIncre = 50; //Default increase/dececrease of the speed of flywheel
    public boolean incremented = false;

    private double flyXOffset = 4;
    private double flyYOffset = 12;
    private double targetXOffset = 6;
    private double targetYOffset = 6;
    private double flyEfficency = 0.5;
    private double flyAngle = 55;
    private double maxRPM = 6000;
    private double flywheelDiameter = 2.5;

    private CRServo[] pRoll = new CRServo[2]; //Purple Ball Rollers
    private CRServo[] gRoll = new CRServo[2]; //Green Ball Rollers
    private double rollSpeed = 0.75; //Speed of the rollers
    private boolean intakeOn = false; //True = on
    private double intakeSpeed = 1; //Speed of intake

    private HuskyLens huskyLens;
    private Limelight3A limelight;
    LLResult llResult;
    private IMU imu;

    //Goal PID
    public static PIDFCoefficients goalPIDCoefficients = new PIDFCoefficients(0.3,0,0,0);
    private PIDFController goalPID = new PIDFController(goalPIDCoefficients);


    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.map = hardwareMap;

        //Flywheels
        bFly = hardwareMap.get(DcMotorEx.class, "pFly");//0 E (Port 0, Expansion Hub Motors)
        tFly = hardwareMap.get(DcMotorEx.class, "gFly");//1 E

        //Rollers
        pRoll[0] = hardwareMap.get(CRServo.class, "pR1");//1 ES
        pRoll[1] = hardwareMap.get(CRServo.class, "pR2");//2 ES
        gRoll[0] = hardwareMap.get(CRServo.class, "gR1");//5 C
        gRoll[1] = hardwareMap.get(CRServo.class, "gR2");//0 C

        //Intake
        intake = hardwareMap.get(DcMotor.class, "intake");//3 E
        sort = hardwareMap.get(Servo.class, "sort");//3 ES (Port 3, Expansion Hub Servo Slots)
        sort.setDirection(Servo.Direction.FORWARD);

        intake.setDirection(DcMotor.Direction.REVERSE);
        bFly.setDirection(DcMotorEx.Direction.FORWARD);
        tFly.setDirection(DcMotorEx.Direction.FORWARD);
        for (CRServo servo:
             pRoll) {
            servo.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        for (CRServo servo:
                gRoll) {
            servo.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bFly.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        tFly.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

       intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bFly.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        tFly.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        //Huksy Lens
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);// Pipelight for April Tags
        limelight.start();

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

    boolean trigger = true; //false is held dwon

    public boolean triggerAsButton(double news){
        double analogThreshold = 0.25;
        return Math.abs(news) > analogThreshold;
    }

    boolean lastTrigger = false;
    public boolean triggerAsButtonPress(double triggerVal){
        double analogThreshold = 0.25;
        boolean current = triggerVal > analogThreshold;
        boolean pressed = current && !lastTrigger;
        lastTrigger = current;
        return pressed;
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

//    //Flywheels
    public void flyPower() {
        if(!fly){
            bFly.setVelocity(flySpeed);
            tFly.setVelocity(flySpeed);
            fly = true;
        } else {
            bFly.setVelocity(flySpeed/4);
            tFly.setVelocity(flySpeed/4);
            fly = false;
        }
    } //Turn on and off power of flywheel

    public void adjustFlySpeed(){
        if(fly){
            bFly.setVelocity(flySpeed);
            tFly.setVelocity(flySpeed);
        }
    }

    public void flyPower(boolean manual) { //True is on, false is off
        //if(fly != manual){ //Only go forward if manual isn't the same as fly
            if(manual){//True is on
                bFly.setVelocity(flySpeed);
                tFly.setVelocity(flySpeed);
                fly = true;
            } else {
                bFly.setVelocity(flySpeed/4);
                tFly.setVelocity(flySpeed/4);
                fly = false;
            }
        //}
    } //Turn on and off power of flywheel based off of what the input is

    public void outtake(int color) {
            if(color == 1) { //Purple
                for (CRServo servo : pRoll) {
                    servo.setPower(rollSpeed);
                }
            } else if (color == 2) { // Green
                for (CRServo servo : gRoll) {
                    servo.setPower(rollSpeed);
                }
            } else if (color == 0) { // Turn off
                for (CRServo servo : pRoll) {
                    servo.setPower(0);
                }
                for (CRServo servo : gRoll) {
                    servo.setPower(0);
                }
            }
    } //Turn on power if needed and spin rollers based on the color

    public void outtakeByCode() {
        int totLoad = 0;
        int totCode = 0;
        for (int l = 0; l < Globals.loaded.length; l++) {
            if(Globals.loaded[l] != 0) {totLoad += Globals.loaded[l];}
            if(Globals.code[l] != 0) { totCode += Globals.code[l];}
        }

       //if(totLoad == totCode){
            flyPower(true);
            for (int color:Globals.code) {
                sleep(5000);
                outtake(color);

                sleep(1250); //Wait 500ms then do the next one
                outtake(0);
            }
            flyPower(false);
        //}

    } //Automatically shoots by the code

    //Intake
    public void intakePower() {
        if(!intakeOn){
            intake.setPower(intakeSpeed);
            intakeOn = true;
        } else {
            intake.setPower(0);
            intakeOn = false;
        }
    } //Turn on and off power of intake

    public void intakePower(boolean manual) { //True is on, false is off
        if(intakeOn != manual){ //Only go forward if manual isn't the same as fly
            if(manual){//True is on
                intake.setPower(intakeSpeed);
                intakeOn = true;
            } else {
                intake.setPower(0);
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

    public void autoSorting() {
        if(!sortOn){
            sortOn = true;
        } else {
            sortOn = false;
        }
    } //Turn on and off power of intake

    public void autoSorting(boolean manual) { //True is on, false is off
        if(sortOn != manual){ //Only go forward if manual isn't the same as fly
            if(manual){//True is on
                sortOn = true;
            } else {
                sortOn = false;
            }
        }
    } //Turn on and off power of intake based off of what the input is


    public void sort() {
        if(sortOn){
            intakePower(true);

            int color = closestColor();

            if(color == 1){ // purple
                sort.setPosition(sortMid+sortInc);
            }else if (color == 2){ //green
                sort.setPosition(sortMid-sortInc);
            }
        } else {
            sort(0);
        }
    }

    public void sort(int color) {
        if (color == 0){
            sort.setPosition(sortMid);
        } else {
            intakePower(true);

            if(color == 1){
                sort.setPosition(sortMid+sortInc);
            }else if (color == 2){
                sort.setPosition(sortMid-sortInc);
            }
        }
    } //0 = No ball, 1 = Purple, 2 = Green

    //Limelight
    public void getLimelight(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        llResult = limelight.getLatestResult();
    }

    public void readFieldData() {
        int codeID = 0;
        int allianceID = 0;

        //Get data
        getLimelight();
        Pose3D botPose = null;
        if(llResult != null && llResult.isValid()){
            botPose = llResult.getBotpose(); //Get position in relation to april tag (i think 90 is straight on)
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());

            for (int i = 0; i < llResult.getFiducialResults().size()-1; i++) {
                for (int h = 0; h < codeIDs.length; h++) {
                    if (codeIDs[h] == llResult.getFiducialResults().get(i).getFiducialId()) {
                        codeID = h;
                        if (Globals.code[0] == 0){
                            Globals.code = codes[codeID];
                        }
                        break;// return index when found
                    } else {
                        allianceID = llResult.getFiducialResults().get(i).getFiducialId();
                        if(Globals.alliance == 0){
                            if(allianceID == 20){Globals.alliance = 2;} else if (allianceID== 24){Globals.alliance = 1;}//Set to opposite color than what it sees
                        }
                    }
                }
            }

            //codeID = llResult.getFiducialResults().get(0).getFiducialId();

            //allianceID = llResult.getFiducialResults().get(1).getFiducialId();//Get second april tag, should be alliance tag
        }

//        for (int i = 0; i < codeIDs.length; i++) {
//            if (codeIDs[i] == codeID) {
//                codeID = i;
//                break;// return index when found
//            }
//        }

//        if (Globals.code[0] == 0){
//            Globals.code = codes[codeID];
//        }
//
//        if(Globals.alliance == 0){
//            if(allianceID == 20){Globals.alliance = 1;} else if (allianceID== 24){Globals.alliance = 2;}
//        }

        if(opMode && botPose != null){
            Globals.startPose = new Pose(
                    botPose.getPosition().x,
                    botPose.getPosition().y,
                    botPose.getOrientation().getYaw(), FTCCoordinates.INSTANCE
            );
        }
    } //Sets the code

    public void faceGoal(){
        if(!facingGoal){
            facingGoal = true;
            goalPID.setTargetPosition(0);
            goalPID.reset();
        } else {
            facingGoal = false;
        }
    }
    public double faceGoalPower() {
            getLimelight();
            Pose3D botPose = null;
            if(llResult != null && llResult.isValid() && facingGoal){
                for (LLResultTypes.FiducialResult fiducial:llResult.getFiducialResults()) {
                    if (fiducial.getFiducialId() != codeIDs[0] && fiducial.getFiducialId() != codeIDs[1] && fiducial.getFiducialId() != codeIDs[2]) {
                        botPose = fiducial.getTargetPoseRobotSpace();
                    }
                }

                if(botPose == null){return 0;}//April tag for tracking is nonexistent or not in camera

                goalPID.updatePosition(Math.atan2(botPose.getPosition().z,botPose.getPosition().x));
                telemetry.addLine("Goal PID Error: " + goalPID.getError());
                return goalPID.run();
            } else {return 0;}
    }

    public void estimatePower() {
        //Get data
        getLimelight();
        Pose3D botPose = null;
        if(llResult != null && llResult.isValid()){
            for (LLResultTypes.FiducialResult fiducial:llResult.getFiducialResults()) {
                if (fiducial.getFiducialId() != codeIDs[0] && fiducial.getFiducialId() != codeIDs[1] && fiducial.getFiducialId() != codeIDs[2]) {
                    botPose = fiducial.getTargetPoseRobotSpace();
                }
            }

            //botPose = llResult.getBotpose();
            if (botPose != null){
                telemetry.addData("Tx", botPose.getPosition().x);
                telemetry.addData("Ty", botPose.getPosition().y);
                telemetry.addData("Tz", botPose.getPosition().z);
                telemetry.addData("Ta", llResult.getTa());

                double distance = Math.sqrt(Math.pow((botPose.getPosition().x*39.3701) + targetXOffset, 2) + Math.pow((botPose.getPosition().z*39.3701) + targetXOffset, 2));//The number added to z is the height the april tag is below the top of the opening in mm i think

                double g = 386.09;
                double velocity = Math.sqrt((g*Math.pow(distance-flyXOffset,2))/(2*Math.pow(Math.cos(Math.toRadians(flyAngle)),2)*(((distance-flyXOffset)*Math.tan(Math.toRadians(flyAngle)))-((botPose.getPosition().y*39.3701)-flyYOffset))));
                double angularVelocity = (velocity)/(flywheelDiameter/2);

                flySpeed = Math.abs(angularVelocity*flyEfficency);
            }
        }
    } //Find distance and get needed power

    //Face goal PID (HELL)

    public void startDrivetrain(){
        drivetrain = new Drivetrain(map,telemetry);
    }

    public void start(){
        readFieldData();
    } //Put things to do in the loop here

    public void update(){
       if (drivetrain != null){
           drivetrain.auxInputs[2] = faceGoalPower();
       }

    } //Put things to do in the loop here
}
