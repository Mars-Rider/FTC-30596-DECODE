package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

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

import org.firstinspires.ftc.teamcode.Swerve.Swerve;

public class Robot {
    public HardwareMap map;
    public final Telemetry telemetry;

    public DcMotor LRL, LFB, RRL, RFB;
    public DcMotor bFly, tFly, intake;
    public Servo sort;

    public boolean opMode = false; //True is auton

    private double sortMid = 0.5;
    private double sortInc = 0.2;

    public double driveSpeed = 0.5; //Default speed of drivetrain
    public double driveSpeedSlow = 0.1; //Speed of drivetrain when in slow mode
    public boolean facingGoal = false; //If true, robot is facing the goal at all times and controls turn into global x and y, False is no more facing goal and it uses local driving

    //static public int alliance = 0; //0 = N/A, 1 = Blue, 2 = Red
    private int[] codeIDs = {21 , 22 , 23}; //Put the ids for each code here
    private int[][] codes = {{2,1,1},{1,2,1},{1,1,2}};
    //static public int[] code = {4,4,4}; //0 = No ball, 1 = Purple, 2 = Green
    // public int[] loaded = {0, 0, 0}; //Order of balls that are loaded - 0 = No ball, 1 = Purple, 2 = Green

    private boolean fly = false; //True = on
    private double dFlySpeed = 0.5; //Default speed of flywheel
    public double flySpeed = 0.45; //Speed of flywheel
    public double flySpeedIncre = 0.05; //Default increase/dececrease of the speed of flywheel
    public boolean incremented = false;

    private double flyXOffset = 0;
    private double flyYOffset = 0;
    private double targetXOffset = 6;
    private double targetYOffset = 6;
    private double flyEfficency = 0.5;
    private double flyAngle = 62.5;
    private double maxRPM = 6000;
    private double flywheelDiameter = 2.5;

    private CRServo[] pRoll = new CRServo[2]; //Purple Ball Rollers
    private CRServo[] gRoll = new CRServo[2]; //Green Ball Rollers
    private double rollSpeed = 0.75; //Speed of the rollers
    private boolean intakeOn = false; //True = on
    private double intakeSpeed = 0.5; //Speed of intake

    //Swerve Stuff
    public double rotationOffset = 0.1;
    private double width = 10;//Inches appart
    private double length = 10; //Inches Infront of CG
    private double radius = Math.sqrt((width*width)+(length+length));

    private HuskyLens huskyLens;
    private Limelight3A limelight;
    LLResult llResult;
    private IMU imu;

    //Goal PID
    private PIDFCoefficients goalPIDCoefficients = new PIDFCoefficients(0.3,0,0,0);
    private PIDFController goalPID = new PIDFController(goalPIDCoefficients);

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.map = hardwareMap;

        //Drivetrain
        LRL = hardwareMap.get(DcMotor.class, "LRL");//2 C (Port 2, Control Hub Motors)
        LFB = hardwareMap.get(DcMotor.class, "LFB");//0 C
        RRL = hardwareMap.get(DcMotor.class, "RRL");//3 C
        RFB = hardwareMap.get(DcMotor.class, "RFB");//1 C

        //Flywheels
        bFly = hardwareMap.get(DcMotor.class, "pFly");//0 E (Port 0, Expansion Hub Motors)
        tFly = hardwareMap.get(DcMotor.class, "gFly");//1 E

        //Rollers
        pRoll[0] = hardwareMap.get(CRServo.class, "pR1");//1 ES
        pRoll[1] = hardwareMap.get(CRServo.class, "pR2");//2 ES
        gRoll[0] = hardwareMap.get(CRServo.class, "gR1");//5 C
        gRoll[1] = hardwareMap.get(CRServo.class, "gR2");//0 C

        //Intake
        intake = hardwareMap.get(DcMotor.class, "intake");//3 E
        sort = hardwareMap.get(Servo.class, "sort");//3 ES (Port 3, Expansion Hub Servo Slots)
        sort.setDirection(Servo.Direction.FORWARD);

        LRL.setDirection(DcMotor.Direction.FORWARD);
        LFB.setDirection(DcMotor.Direction.FORWARD);
        RRL.setDirection(DcMotor.Direction.REVERSE);
        RFB.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        bFly.setDirection(DcMotor.Direction.FORWARD);
        tFly.setDirection(DcMotor.Direction.REVERSE);
        for (CRServo servo:
             pRoll) {
            servo.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        for (CRServo servo:
                gRoll) {
            servo.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        RRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LFB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bFly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        tFly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RRL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LRL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        tFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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

    public boolean triggerAsButtonPress(double news){
        double analogThreshold = 0.25;
        if(trigger == true && Math.abs(news) > analogThreshold){
            trigger = false;
            return true;
        } else if (Math.abs(news) < analogThreshold){
            trigger = true;
            return false;
        }
        return false;
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
    public void flyPower() {
        if(!fly){
            bFly.setPower(flySpeed);
            tFly.setPower(flySpeed);
            fly = true;
        } else {
            bFly.setPower(0);
            tFly.setPower(0);
            fly = false;
        }
    } //Turn on and off power of flywheel

    public void adjustFlySpeed(){
        if(fly){
            bFly.setPower(flySpeed);
            tFly.setPower(flySpeed);
        }
    }

    public void flyPower(boolean manual) { //True is on, false is off
        //if(fly != manual){ //Only go forward if manual isn't the same as fly
            if(manual){//True is on
                bFly.setPower(flySpeed);
                tFly.setPower(flySpeed);
                fly = true;
            } else {
                bFly.setPower(0);
                tFly.setPower(0);
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

        if(totLoad == totCode){
            for (int color:Globals.code) {
                outtake(color);

                sleep(500); //Wait 500ms then do the next one
            }
        }

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

    public void sort() {
        intakePower(true);

        int color = closestColor();

        if(color == 1){ // purple
            sort.setPosition(sortMid+sortInc);
            return;
        }else if (color == 2){ //green
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
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        llResult = limelight.getLatestResult();
        Pose3D botPose = null;
        if(llResult != null && llResult.isValid()){
            botPose = llResult.getBotpose(); //Get position in relation to april tag (i think 90 is straight on)
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());

            for (int i = 0; i < llResult.getFiducialResults().size(); i++) {
                for (int h = 0; i < codeIDs.length; h++) {
                    if (codeIDs[h] == codeID) {
                        codeID = i;
                        if (Globals.code[0] == 0){
                            Globals.code = codes[codeID];
                        }
                        break;// return index when found
                    } else {
                        if(Globals.alliance == 0){
                            if(allianceID == 20){Globals.alliance = 1;} else if (allianceID== 24){Globals.alliance = 2;}
                        }
                    }
                }
            }

            //codeID = llResult.getFiducialResults().get(0).getFiducialId();

            //allianceID = llResult.getFiducialResults().get(1).getFiducialId();//Get second april tag, should be alliance tag
        }

        for (int i = 0; i < codeIDs.length; i++) {
            if (codeIDs[i] == codeID) {
                codeID = i;
                break;// return index when found
            }
        }

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


    public double faceGoalPower() {
        //Pedro
        return 0;
    }//Track april tag
    public void estimatePower() {
        //Get data
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        llResult = limelight.getLatestResult();
        Pose3D botPose = null;
        if(llResult != null && llResult.isValid()){
            for (LLResultTypes.FiducialResult fiducial:llResult.getFiducialResults()) {
                if (fiducial.getFiducialId() == 20 && Globals.alliance == 1) {
                    //botPose = fiducial.getTargetPoseRobotSpace();
                }
            }

            botPose = llResult.getBotpose();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());

            double distance = Math.sqrt(Math.pow(botPose.getPosition().x+targetXOffset, 2) + Math.pow(botPose.getPosition().y+targetXOffset, 2));//The number added to z is the height the april tag is below the top of the opening in mm i think

            double g = 386.09;
            double velocity = Math.sqrt((g*Math.pow(distance-flyXOffset,2))/(2*Math.pow(Math.cos(Math.toRadians(flyAngle)),2)*(((distance-flyXOffset)*Math.tan(Math.toRadians(flyAngle)))-(distance-flyYOffset))));
            double rpm = (60*velocity)/(flywheelDiameter*Math.toRadians(180));
            double power = rpm/(maxRPM*flyEfficency);

            flySpeed = Math.abs(power);
        }
    } //Find distance and get needed power

    //Face goal PID (HELL)

    public void SwerveDrive(double x, double y, double r){

        r= r/(Math.abs(r)+Math.abs(rotationOffset));

        double B = x + (r*(length/radius));
        double C = y - (r*(width/radius));
        double D = y + (r*(width/radius));
        double m = Math.max(1, Math.max(Math.sqrt((B*B)+(C*C)),Math.sqrt((B*B)+(D*D))));

        Vector[] movementVectors = new Vector[]{
                new Vector(Math.sqrt((B*B)+(D*D))/m, Math.atan2(B,D)), //Left (Swap B and D as first is the y and second is x, and I think that B is using a x compontent) (B,D) bassically = (X,Y), but the function is meant for (y,x)
                new Vector(Math.sqrt((B*B)+(C*C))/m, Math.atan2(B,C)) // Right
        };

        // the powers for the wheel vectors
        double[] wheelPowers = new double[4];

        movementVectors = Swerve.swerve(x,y,r);

        wheelPowers[0] = movementVectors[0].getYComponent();
        wheelPowers[1] = movementVectors[0].getXComponent();
        wheelPowers[2] = movementVectors[1].getYComponent();
        wheelPowers[3] = movementVectors[1].getXComponent();


        LFB.setPower(wheelPowers[0]);
        LRL.setPower(wheelPowers[1]);
        RFB.setPower(wheelPowers[2]);
        RRL.setPower(wheelPowers[3]);
    }

    public void drive(double x, double y, double r){
        double[] wheelPowers = {0,0,0,0}; //Fr, fl, br, bl

        wheelPowers[0] = y-x-r;
        wheelPowers[1] = y+x+r;
        wheelPowers[2] = y-x+r;
        wheelPowers[3] = y+x-r;

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

    public void start(){
        readFieldData();
    } //Put things to do in the loop here

    public void update(){
        incremented = false;

        sort();
    } //Put things to do in the loop here
}
