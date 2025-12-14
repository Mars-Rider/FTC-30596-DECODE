package org.firstinspires.ftc.teamcode.Robot;

import static android.os.SystemClock.sleep;

import android.graphics.Color;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
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

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Robot.LEDs.LEDController;
import org.firstinspires.ftc.teamcode.Robot.LEDs.LEDController.LEDChannel;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;


public class Robot {
    public HardwareMap map;
    public final Telemetry telemetry;

    public Drivetrain drivetrain;

    public DcMotorEx bFly, tFly;
    public DcMotor intake;
    public Servo sort;

    public VoltageSensor voltageSensor;

    public LEDController LEDs;
    public LEDChannel flyLEDs;

    public boolean sortOn = false;//True is sorting

    public boolean facingGoal = false; //If true, robot is facing the goal at all times and controls turn into global x and y, False is no more facing goal and it uses local driving

    //static public int alliance = 0; //0 = N/A, 1 = Blue, 2 = Red
    private int[] codeIDs = {21 , 22 , 23}; //Put the ids for each code here
    private int[][] codes = {{2,1,1},{1,2,1},{1,1,2}};
    //static public int[] code = {4,4,4}; //0 = No ball, 1 = Purple, 2 = Green
    // public int[] loaded = {0, 0, 0}; //Order of balls that are loaded - 0 = No ball, 1 = Purple, 2 = Green

    private boolean fly = false; //True = on
    public double flySpeed = 67.5;
    //public double flySpeed = 0.65;

    public boolean incremented = false;

    private CRServo[] pRoll = new CRServo[2]; //Purple Ball Rollers
    private CRServo[] gRoll = new CRServo[2]; //Green Ball Rollers
    private double rollSpeed = 0.75; //Speed of the rollers
    private boolean intakeOn = false; //True = on
    private double intakeSpeed = 1; //Speed of intake
    public int intakeDirection = 1;

    public NormalizedColorSensor[] pColor = new NormalizedColorSensor[2];
    public NormalizedColorSensor[] gColor = new NormalizedColorSensor[2];
    private float gain = 2;
    private final float[] hsvValues = new float[3];
    private final int[] pDomain = {250,320};
    private final int[] gDomain = {75,165};


    private HuskyLens huskyLens;
    private Limelight3A limelight;
    LLResult llResult;
    private IMU imu;

    //Goal PID
    public double error;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.map = hardwareMap;

        //Flywheels
        bFly = hardwareMap.get(DcMotorEx.class, "pFly");//0 E (Port 0, Expansion Hub Motors)
        tFly = hardwareMap.get(DcMotorEx.class, "gFly");//1 E

        //Rollers
        pRoll[0] = hardwareMap.get(CRServo.class, "pR1");//1 ES        pRoll[1] = hardwareMap.get(CRServo.class, "pR2");//2 ES
        pRoll[1] = hardwareMap.get(CRServo.class, "pR2");//2 ES
        gRoll[0] = hardwareMap.get(CRServo.class, "gR1");//5 C
        gRoll[1] = hardwareMap.get(CRServo.class, "gR2");//0 C

        //Color Sensors
        pColor[0] = hardwareMap.get(NormalizedColorSensor.class, "pColor1");
        pColor[1] = hardwareMap.get(NormalizedColorSensor.class, "pColor2");
        gColor[0] = hardwareMap.get(NormalizedColorSensor.class, "gColor1");
        gColor[1] = hardwareMap.get(NormalizedColorSensor.class, "gColor2");

        for (NormalizedColorSensor colorSensor:
                pColor) {
            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight)colorSensor).enableLight(true);
            }
        }
        for (NormalizedColorSensor colorSensor:
                gColor) {
            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight)colorSensor).enableLight(true);
            }
        }

        //Intake
        intake = hardwareMap.get(DcMotor.class, "intake");//3 E
        sort = hardwareMap.get(Servo.class, "sort");//3 ES (Port 3, Expansion Hub Servo Slots)
        sort.setDirection(Servo.Direction.FORWARD);

        intake.setDirection(DcMotor.Direction.REVERSE);
        bFly.setDirection(DcMotorEx.Direction.REVERSE);
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
        //bFly.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        MotorConfigurationType motorType = bFly.getMotorType().clone();
        motorType.setTicksPerRev(112);  // your real encoder count
        bFly.setMotorType(motorType);
        bFly.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bFly.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        tFly.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

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

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        //LEDs = new LEDController(hardwareMap,"leds");
        //flyLEDs = LEDs.addChannel("flyLEDs");
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



    //Color sensors
    public int color(NormalizedColorSensor colorSensor){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0]);

        if(hsvValues[0] > pDomain[0] && hsvValues[0] < pDomain[1]){//Less than ??? means
            return 1;
        } else if (hsvValues[0] > gDomain[0] && hsvValues[0] < gDomain[1]){
            return 2;
        } else {
            return 0;
        }
    }

    public void waitForEnter(NormalizedColorSensor colorSensor){//Wait for color to leave the sensor
        while (color(colorSensor) == 0){sleep(50);}
    }

    public void waitForLeave(NormalizedColorSensor colorSensor){//Wait for color to leave the sensor
        while (color(colorSensor) != 0){sleep(50);}
    }

    public void load(int color){//Add a color to the load
        for (int i = 0; i < 3; i++) {
            if(Globals.loaded[i] == 0){Globals.loaded[i] = color; break;}
        }
    }

    public void unLoad(int color){
        for (int i = 2; i >= 0; i--) {//Make sure it doesnt go to index of 3
            if(Globals.loaded[i] == color){Globals.loaded[i] = 0; break;}
        }
    }

    //Flywheels
    public void flyPower() {
        flyPower(!fly);
    } //Turn on and off power of flywheel

    public void adjustFlySpeed(){
        if(fly){
            flyPower(fly);
        }
    }

    public void flyPower(boolean manual) { //True is on, false is off
        if(manual){//True is on
                bFly.setVelocity(flySpeed, AngleUnit.RADIANS);
                tFly.setVelocity(flySpeed, AngleUnit.RADIANS);
//            bFly.setPower(flySpeed, AngleUnit.RADIANS);
//            tFly.setPower(flySpeed);
            fly = true;
        } else {
                bFly.setVelocity(0);
                tFly.setVelocity(0);
//            bFly.setPower(0);
//            tFly.setPower(0);
            fly = false;
        }
    } //Turn on and off power of flywheel based off of what the input is

    private boolean pOut = false, gOut = false;
    public void outtake(int color) {
            if(color == 1) { //Purple
                if(!pOut){
                    for (CRServo servo : pRoll) {
                        servo.setPower(rollSpeed);
                    }
                    pOut = true;
                } else {
                    for (CRServo servo : pRoll) {
                        servo.setPower(0);
                    }
                    pOut = false;
                }
            } else if (color == 2) { // Green
                if(!gOut){
                    for (CRServo servo : gRoll) {
                        servo.setPower(rollSpeed);
                    }
                    gOut = true;
                } else {
                    for (CRServo servo : gRoll) {
                        servo.setPower(0);
                    }
                    gOut = false;
                }
            } else if (color == 0) { // Turn off
                for (CRServo servo : pRoll) {
                    servo.setPower(0);
                }
                for (CRServo servo : gRoll) {
                    servo.setPower(0);
                }

                pOut = false;
                gOut = false;
            }
    } //Turn on power if needed and spin rollers based on the color

    public void setOuttake(int color, boolean state){
        if(color == 1) { //Purple
            if(state){
                for (CRServo servo : pRoll) {
                    servo.setPower(rollSpeed);
                }
                pOut = true;
            } else {
                for (CRServo servo : pRoll) {
                    servo.setPower(0);
                }
                pOut = false;
            }
        } else if (color == 2) { // Green
            if(state){
                for (CRServo servo : gRoll) {
                    servo.setPower(rollSpeed);
                }
                gOut = true;
            } else {
                for (CRServo servo : gRoll) {
                    servo.setPower(0);
                }
                gOut = false;
            }
        }
    }

    public void outtakeByCode() {

        outtakeByCode(Globals.code);

    } //Automatically shoots by the code

    public void sleepForFly(){
        sleepForFly(0);
    }

    public void sleepForFly(double delay){
        while (Math.abs(bFly.getVelocity(AngleUnit.RADIANS)-flySpeed+delay) > Globals.flyTolerance) {
            sleep(50);
            telemetry.addLine("Error: " + Math.abs(bFly.getVelocity(AngleUnit.RADIANS)-flySpeed));
            telemetry.update();
        }
    }

    public void outtakeByCode(int[] code) {
//        int totLoad = 0;
//        int totCode = 0;
//
//        for (int l = 0; l < Globals.loaded.length; l++) {
//            if(Globals.loaded[l] != 0) {totLoad += Globals.loaded[l];}
//            if(code[l] != 0) { totCode += code[l];}
//        }

        //if(totLoad == totCode){
        //}

        //if(totLoad == totCode){
        if(code[0] == 0){
            code = new int[]{1,1,2};
        }

        if(!fly){
            flyPower(true);
            //waitForFly();
            sleepForFly(-2.5);
        }
        for (int color:code) {
            //waitForFly(); //before using, Disable all the sleeps in this function but the one between the outtake function calls
            outtake(color);
            sleep(1250); //Wait 500ms then do the next one
            outtake(0);

            sleepForFly(); //Wait 500ms then do the next one - Change to wait until fly wheel is ready
        }
        sleep(250);
        flyPower(false);
        //}

    } //Automat

    //Intake
    public void intakePower() {
        intakePower(!intakeOn);
    } //Turn on and off power of intake

    public void intakePower(boolean manual) { //True is on, false is off
            if(manual){//True is on
                intake.setPower(intakeDirection*intakeSpeed);
                intakeOn = true;
            } else {
                intake.setPower(0);
                intakeOn = false;
            }
    } //Turn on and off power of intake based off of what the input is

    int lastColor;
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
        autoSorting(!sortOn);
    } //Turn on and off power of intake
    public void autoSorting(boolean manual) { //True is on, false is off
        sortOn = manual;
    } //Turn on and off power of intake based off of what the input is


    public void sort() {
        if(sortOn){
            sort(closestColor());
        } else {
            sort(0);
        }
    }
    public void sort(int color) {
        if (color == 0){
            sort.setPosition(Settings.sortMid);
        } else {
            intakePower(true);

            if(color == 1){
                sort.setPosition(Settings.sortMid+Settings.sortInc);
            }else if (color == 2){
                sort.setPosition(Settings.sortMid-Settings.sortInc);
            }
        }
    } //0 = No ball, 1 = Purple, 2 = Green

    //Limelight
    public void getLimelight(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        llResult = limelight.getLatestResult();
    }

    public void resetCode(){
        Globals.code = new int[]{0,0,0};


    }

    public void readFieldData() {readFieldData(false);}

    public void readFieldData(boolean reset) {
        int codeID = 0;
        int allianceID = 0;

        if(reset){
            Globals.code = new int[]{0,0,0};
            Globals.alliance = 0;
        }

        //Get data
        getLimelight();
        Pose3D botPose = null;
        if(llResult != null && llResult.isValid()){
            botPose = llResult.getBotpose(); //Get position in relation to april tag (i think 90 is straight on)
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());

            for (int i = 0; i < llResult.getFiducialResults().size()-1; i++) {
                telemetry.addData("Code Id", llResult.getFiducialResults().get(i).getFiducialId());
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

            if (Globals.alliance == 0){
                if (botPose.getPosition().y < 0){
                    Globals.alliance = 1; //Blue?
                } else {
                    Globals.alliance = 2; //Red?
                }
            }

            //codeID = llResult.getFiducialResults().get(0).getFiducialId();

            //allianceID = llResult.getFiducialResults().get(1).getFiducialId();//Get second april tag, should be alliance tag
        }

        if(Globals.opMode && botPose != null){
            Globals.startPose = new Pose(
                    (botPose.getPosition().y + 1.8288) * 39.3701,//Switch depending on the thing
                    (-botPose.getPosition().x + 1.8288) * 39.3701,
                    Math.toRadians(botPose.getOrientation().getYaw()+90) //Adjust if the yaw is wrong
            );
        }
    } //Sets the code

    public void faceGoal(){
        drivetrain.runTo(faceGoalError(false));
    }

    public double faceGoalError(){
        return faceGoalError(false);
    }

    boolean headingSwap = false;

    public double faceGoalError(boolean robotCentric){
        //use pedro to find if the lime light can see it
        Pose goalPose = (Globals.alliance == 1) ? new Pose(16,130) : new Pose(128,130);
        error = (Math.atan2(goalPose.getY()-drivetrain.getPosition().getY(),goalPose.getX()-drivetrain.getPosition().getX())-Math.toRadians(180)) - (drivetrain.getHeading());
        error = error % (2*Math.PI);//Make it normalized

        telemetry.addLine("Pedro Error: " + Math.toDegrees(-error));

        if(Math.abs(Math.toDegrees(error)) <= 30){ //If it not greater than limelight field of view

            getLimelight();
            Pose3D botPose = null;
            if(llResult != null && llResult.isValid()){
                for (LLResultTypes.FiducialResult fiducial:llResult.getFiducialResults()) {
                    if (fiducial.getFiducialId() != codeIDs[0] && fiducial.getFiducialId() != codeIDs[1] && fiducial.getFiducialId() != codeIDs[2]) {
                        botPose = fiducial.getTargetPoseRobotSpace();
                    }
                }

                if(botPose != null){//April tag for tracking is nonexistent or not in camera
                    error = error % (2*Math.PI);//Make it normalized
                    telemetry.addLine("Limelight Error: " + Math.toDegrees(-error));
                } else {
                    telemetry.addLine("Using limelight, no tag");
                }
            } else {
                telemetry.addLine("Using limelight, no result");
                //Robot is either facing the wrong way or too far from limelight
                //return faceGoalError();
            }
        }


        telemetry.addLine("Goal Error: " + Math.toDegrees(-error));

        if(Math.abs(Math.toDegrees(error)) < Settings.goalDeadzone) {error = 0;}

        return robotCentric ? -error : error + drivetrain.getHeading();
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

                double distance = (botPose.getPosition().z*39.3701) + Settings.powerEstimate.targetXOffset;//The number added to z is the height the april tag is below the top of the opening in mm i think

                double g = 386.09;
                double velocity = Math.sqrt((g*Math.pow(distance-Settings.powerEstimate.flyXOffset,2))/(2*Math.pow(Math.cos(Math.toRadians(Settings.powerEstimate.flyAngle)),2)*((((distance-Settings.powerEstimate.flyXOffset)*Math.tan(Math.toRadians(Settings.powerEstimate.flyAngle))))-((botPose.getPosition().y*39.3701)+Settings.powerEstimate.targetYOffset-Settings.powerEstimate.flyYOffset))));
                double angularVelocity = (velocity)/(Settings.powerEstimate.flywheelDiameter/2);//Gets it in radians/sec
                double rpm = (60*velocity)/(Settings.powerEstimate.flywheelDiameter*Math.toRadians(180));

                flySpeed = Math.abs((angularVelocity)*(1/Settings.powerEstimate.flyEfficency))*Settings.powerEstimate.flyOverEstimate;
            }
        }
    } //Find distance and get needed power


    public void startDrivetrain(){
        drivetrain = new Drivetrain(map,telemetry);
    }

    public void startPedroDrivetrain(){
        drivetrain = new Drivetrain(map,telemetry, true);
    }

    public void start(){

    }

    public void update(){
        //drivetrain.auxInputs[2] = drivetrain != null && facingGoal ? faceGoalPower() : 0;

        if(Math.abs(bFly.getVelocity(AngleUnit.RADIANS)-flySpeed) < Globals.flyTolerance){
            //flyLEDs.setColor(Color.GREEN);
            telemetry.addLine("Fly Wheel Ready");
        } else {
            //flyLEDs.setColor(Color.BLUE);
        }

        drivetrain.update();
    } //Put things to do in the loop here

    public static double changeAlliance(double input){
        if(Globals.alliance == 2){
            return (140-input);
        } else {
            return input;
        }
    }
}
