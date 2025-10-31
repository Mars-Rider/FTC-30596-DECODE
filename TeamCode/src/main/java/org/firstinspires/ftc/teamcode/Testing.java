package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@TeleOp
public class Testing extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        //Run at Initialization
        Servo sort;
        HuskyLens huskyLens;

        double sortMid = 0.5;
        double sortInc = 0.2;

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        sort = hardwareMap.get(Servo.class, "sort");//3 ES (Port 3, Expansion Hub Servo Slots)
        sort.setDirection(Servo.Direction.FORWARD);

        waitForStart();
        //On Start

        if (isStopRequested()) return;

        while (opModeIsActive()) {
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

            if(closestColor == 1){
                sort.setPosition(sortMid+sortInc);
                telemetry.addLine("Closest Color: Purple");
            }else if (closestColor == 2){
                sort.setPosition(sortMid-sortInc);
                telemetry.addLine("Closest Color: Green");
            } else {
                telemetry.addLine("Closest Color: N/a");
            }
            telemetry.update();
        }
    }
}