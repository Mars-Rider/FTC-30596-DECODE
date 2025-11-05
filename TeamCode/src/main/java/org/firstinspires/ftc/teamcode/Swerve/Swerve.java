package org.firstinspires.ftc.teamcode.Swerve;

import static org.firstinspires.ftc.teamcode.Swerve.Constants.modules;

import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Swerve.Constants;

public class Swerve {
    public static double[] inputs = {0,0,0};

    private static Vector translationVector(){
        Vector translationVector = new Vector(0,0);
        translationVector.setOrthogonalComponents(inputs[0],inputs[1]);

        return translationVector;
    }

    private static Vector translationVector(double[] module){
        Vector translationVector = new Vector(0,0);
        translationVector.setOrthogonalComponents(module[0]+inputs[0],module[1]+inputs[1]);
        return translationVector;
    }

    public static Vector[] perpendicularVectors(){
        Vector[] perpendicularVectors = new Vector[modules.length];

        for (int i = 0; i < modules.length; i++) {
            perpendicularVectors[i] = new Vector(0,0);
            perpendicularVectors[i].setOrthogonalComponents(-modules[i][1],modules[i][0]);
        }
        return perpendicularVectors;
    }

    private static Vector perpendicularVector(double[] module){
        Vector perpendicularVector = new Vector(0,0);
        perpendicularVector.setOrthogonalComponents(-module[1],module[0]);
        return perpendicularVector;
    }

    private static Vector rotationVector(double[] module){
        Vector perpendicularVector = perpendicularVector(module);
        Vector rotationVector = new Vector(0,0);

        rotationVector.setOrthogonalComponents(inputs[2] * perpendicularVector.getXComponent(), inputs[2] * perpendicularVector.getYComponent());

        return rotationVector;
    }

    private static Vector outputVector(double[] module){
        Vector outputVector = translationVector();
        outputVector = outputVector.plus(rotationVector(module));

        return outputVector;
    }

    private static Vector[] clampVectors(Vector[] vectors){
        double magnitude = 1;

        for (Vector v: vectors) {
            if (v.getMagnitude() > magnitude){
                magnitude = v.getMagnitude();
            }
        }

        for (Vector v: vectors) {
            v.setMagnitude(v.getMagnitude()/magnitude);
        }

        return vectors;
    }

    public static Vector[] swerve(double x, double y, double r){
        inputs[0] = x;
        inputs[1] = y;
        inputs[2] = r;

        Vector[] vectors = new Vector[modules.length];

        for (int i = 0; i < vectors.length; i++) {
            vectors[i] = outputVector(modules[i]);
        }

        return clampVectors(vectors);
    }
}
