package com.qualcomm.ftcrobotcontroller.opmodes.first;

import com.qualcomm.ftcrobotcontroller.opmodes.BasicVisionSample;
import com.qualcomm.ftcrobotcontroller.opmodes.ManualVisionSample;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import org.lasarobotics.vision.test.android.Cameras;
import org.lasarobotics.vision.test.opmode.VisionExtensions;
import org.lasarobotics.vision.test.opmode.VisionOpMode;
import org.opencv.core.Size;

/**
 * Created by heinz on 1/4/2016.
 */
public class ColorCamera extends LinearOpMode {

    DcMotor right_drive;
    DcMotor left_drive;
    DcMotor tread_drive;
    public double centerX = 455;
    public double centerY = 432;

    public void runOpMode() throws InterruptedException {

        ManualVisionSample visionSample = new ManualVisionSample();
        visionSample.init();

        right_drive = hardwareMap.dcMotor.get("right_drive");
        left_drive = hardwareMap.dcMotor.get("left_drive");
        tread_drive = hardwareMap.dcMotor.get("tread_drive");
        //gyro_sensor = hardwareMap.gyroSensor.get("gyro_sensor");

        right_drive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            visionSample.loop();
            // Set initial motor movements
            double beacon_centerx = visionSample.beaconCenterX;
            double beacon_centery = visionSample.beaconCenterY;
            String beacon_color = visionSample.beaconColor;
            String confidence = visionSample.analyConfi;
            Integer confidence_int = Integer.parseInt(confidence);
            if (beacon_centerx < centerX)
            {
                right_drive.setPower(-1.0);
                left_drive.setPower(1.0);
            }
            else if (beacon_centerx > centerX)
            {
                right_drive.setPower(1.0);
                left_drive.setPower(-1.0);
            }
            else {}
//            if (confidence_int > 80) //If analysis confidence is great enough
//            {
//                switch(beacon_color)
//                {
//                    case "red, blue":
//
//                        break;
//                    case "blue, red":
//                        // execute code
//                        break;
//                    default:
//                        //execute code
//                        break;
//                }
//            }
//            else {}
        }

    }

}
