package com.pedropathing.ftc.drivetrains;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class BallConstants {
    /** The Forward Velocity of the Robot - Different for each robot
     *  Default Value: 81.34056 */
    public  double xVelocity = 81.34056;

    /** The Lateral Velocity of the Robot - Different for each robot
     *  Default Value: 65.43028 */
    public  double yVelocity = 65.43028;


    public  double maxPower = 1;
    public  String leftFrontMotorName = "leftFront";
    public  String leftRearMotorName = "leftRear";
    public  String rightFrontMotorName = "rightFront";
    public  String rightRearMotorName = "rightRear";
    public  DcMotorSimple.Direction leftLongMotorDirection = DcMotorSimple.Direction.REVERSE;
    public  DcMotorSimple.Direction leftLatMotorDirection = DcMotorSimple.Direction.REVERSE;
    public  DcMotorSimple.Direction rightLongMotorDirection = DcMotorSimple.Direction.FORWARD;
    public  DcMotorSimple.Direction rightLatMotorDirection = DcMotorSimple.Direction.FORWARD;
    public  double motorCachingThreshold = 0.01;
    public  boolean useBrakeModeInTeleOp = false;
    public  boolean useVoltageCompensation = false;
    public  double nominalVoltage = 12.0;
    public  double staticFrictionCoefficient = 0.1;

    public BallConstants() {
        defaults();
    }

    public BallConstants xVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
        return this;
    }

    public BallConstants yVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
        return this;
    }

    public BallConstants maxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    public BallConstants leftFrontMotorName(String leftFrontMotorName) {
        this.leftFrontMotorName = leftFrontMotorName;
        return this;
    }

    public BallConstants leftRearMotorName(String leftRearMotorName) {
        this.leftRearMotorName = leftRearMotorName;
        return this;
    }

    public BallConstants rightFrontMotorName(String rightFrontMotorName) {
        this.rightFrontMotorName = rightFrontMotorName;
        return this;
    }

    public BallConstants rightRearMotorName(String rightRearMotorName) {
        this.rightRearMotorName = rightRearMotorName;
        return this;
    }

    public BallConstants leftFrontMotorDirection(DcMotorSimple.Direction leftFrontMotorDirection) {
        this.leftLongMotorDirection = leftFrontMotorDirection;
        return this;
    }

    public BallConstants leftRearMotorDirection(DcMotorSimple.Direction leftRearMotorDirection) {
        this.leftLatMotorDirection = leftRearMotorDirection;
        return this;
    }

    public BallConstants rightFrontMotorDirection(DcMotorSimple.Direction rightFrontMotorDirection) {
        this.rightLongMotorDirection = rightFrontMotorDirection;
        return this;
    }

    public BallConstants rightRearMotorDirection(DcMotorSimple.Direction rightRearMotorDirection) {
        this.rightLatMotorDirection = rightRearMotorDirection;
        return this;
    }

    public BallConstants motorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
        return this;
    }

    public BallConstants useBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        this.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
        return this;
    }

    public BallConstants useVoltageCompensation(boolean useVoltageCompensation) {
        this.useVoltageCompensation = useVoltageCompensation;
        return this;
    }

    public BallConstants nominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public BallConstants staticFrictionCoefficient(double staticFrictionCoefficient) {
        this.staticFrictionCoefficient = staticFrictionCoefficient;
        return this;
    }

    public double getXVelocity() {
        return xVelocity;
    }

    public void setXVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
    }

    public double getYVelocity() {
        return yVelocity;
    }

    public void setYVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public String getLeftFrontMotorName() {
        return leftFrontMotorName;
    }

    public void setLeftFrontMotorName(String leftFrontMotorName) {
        this.leftFrontMotorName = leftFrontMotorName;
    }

    public String getLeftRearMotorName() {
        return leftRearMotorName;
    }

    public void setLeftRearMotorName(String leftRearMotorName) {
        this.leftRearMotorName = leftRearMotorName;
    }

    public String getRightFrontMotorName() {
        return rightFrontMotorName;
    }

    public void setRightFrontMotorName(String rightFrontMotorName) {
        this.rightFrontMotorName = rightFrontMotorName;
    }

    public String getRightRearMotorName() {
        return rightRearMotorName;
    }

    public void setRightRearMotorName(String rightRearMotorName) {
        this.rightRearMotorName = rightRearMotorName;
    }

    public DcMotorSimple.Direction getLeftLongMotorDirection() {
        return leftLongMotorDirection;
    }

    public void setLeftLongMotorDirection(DcMotorSimple.Direction leftLongMotorDirection) {
        this.leftLongMotorDirection = leftLongMotorDirection;
    }

    public DcMotorSimple.Direction getLeftLatMotorDirection() {
        return leftLatMotorDirection;
    }

    public void setLeftLatMotorDirection(DcMotorSimple.Direction leftLatMotorDirection) {
        this.leftLatMotorDirection = leftLatMotorDirection;
    }

    public DcMotorSimple.Direction getRightLongMotorDirection() {
        return rightLongMotorDirection;
    }

    public void setRightLongMotorDirection(DcMotorSimple.Direction rightLongMotorDirection) {
        this.rightLongMotorDirection = rightLongMotorDirection;
    }

    public DcMotorSimple.Direction getRightLatMotorDirection() {
        return rightLatMotorDirection;
    }

    public void setRightLatMotorDirection(DcMotorSimple.Direction rightLatMotorDirection) {
        this.rightLatMotorDirection = rightLatMotorDirection;
    }

    public double getMotorCachingThreshold() {
        return motorCachingThreshold;
    }

    public void setMotorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
    }

    public boolean isUseBrakeModeInTeleOp() {
        return useBrakeModeInTeleOp;
    }

    public void setUseBrakeModeInTeleOp(boolean useBrakeModeInTeleOp) {
        this.useBrakeModeInTeleOp = useBrakeModeInTeleOp;
    }

    /**
     * This method sets the default values for the MecanumConstants class.
     * It is called in the constructor of the MecanumConstants class.
     */
    public void defaults() {
        xVelocity = 81.34056;
        yVelocity = 65.43028;
        //convertToPolar = Pose.cartesianToPolar(xVelocity, -yVelocity);
        //frontLeftVector = new Vector(convertToPolar[0], convertToPolar[1]).normalize();
        maxPower = 1;
        leftFrontMotorName = "leftFront";
        leftRearMotorName = "leftRear";
        rightFrontMotorName = "rightFront";
        rightRearMotorName = "rightRear";
        leftLongMotorDirection = DcMotorSimple.Direction.REVERSE;
        leftLatMotorDirection = DcMotorSimple.Direction.REVERSE;
        rightLongMotorDirection = DcMotorSimple.Direction.FORWARD;
        rightLatMotorDirection = DcMotorSimple.Direction.FORWARD;
        motorCachingThreshold = 0.01;
        useBrakeModeInTeleOp = false;
        useVoltageCompensation = false;
        nominalVoltage = 12.0;
        staticFrictionCoefficient = 0.1;
    }
}
