package org.firstinspires.ftc.teamcode.Swerve;

import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Swerve.Constants;

public class Swerve {
    public double[] inputs = {0,0,0};

    private Vector translationVector(){
        Vector translationVector = new Vector(0,0);
        translationVector.setOrthogonalComponents(inputs[0],inputs[1]);

        return translationVector;
    }

    private Vector translationVector(double[] module){
        Vector translationVector = new Vector(0,0);
        translationVector.setOrthogonalComponents(module[0]+inputs[0],module[1]+inputs[1]);
        return translationVector;
    }


    private Vector perpendicularVector(double[] module){
        Vector perpendicularVector = new Vector(0,0);
        perpendicularVector.setOrthogonalComponents(-module[1],module[0]);
        return perpendicularVector;
    }

    private Vector rotationVector(double[] module){
        Vector perpendicularVector = perpendicularVector(module);
        Vector rotationVector = new Vector(0,0);

        rotationVector.setOrthogonalComponents(inputs[2] * perpendicularVector.getXComponent(), inputs[2] * perpendicularVector.getYComponent());

        return rotationVector;
    }

    private Vector outputVector(double[] module){
        Vector outputVector = translationVector();
        outputVector.plus(rotationVector(module));

        return outputVector;
    }

    private Vector[] clampVectors(Vector[] vectors){
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

        Vector[] vectors = new Vector[Constants.modules.length];

        for (int i = 0; i < vectors.length; i++) {
            vectors[i] = outputVector(Constants.modules[i]);
        }

        vectors = clampVectors(vectors);
        return vectors;
    }
}
