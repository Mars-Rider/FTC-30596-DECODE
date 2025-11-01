package org.firstinspires.ftc.teamcode;


import com.pedropathing.geometry.Pose;

//Holds global variables to be used between opModes
public class Globals {

    static public int alliance = 0; //0 = N/A, 1 = Blue, 2 = Red

    static public int[] code = {0,0,0}; //0 = No ball, 1 = Purple, 2 = Green

    static public int[] loaded = {0, 0, 0}; //Order of balls that are loaded - 0 = No ball, 1 = Purple, 2 = Green

    static public Pose startPose;
}
