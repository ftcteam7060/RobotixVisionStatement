package com.qualcomm.ftcrobotcontroller.opmodes.first;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.*;

/**
 * Created by heinz on 1/4/2016.
 */
public class ClimberDeposit extends LinearOpMode {

    DcMotor right_drive;
    DcMotor left_drive;
    DcMotor tread_drive;
    //GyroSensor gyro_sensor;

    public void runOpMode() throws InterruptedException {

        right_drive = hardwareMap.dcMotor.get("right_drive");
        left_drive = hardwareMap.dcMotor.get("left_drive");
        tread_drive = hardwareMap.dcMotor.get("tread_drive");
        //gyro_sensor = hardwareMap.gyroSensor.get("gyro_sensor");

        right_drive.setDirection(DcMotor.Direction.REVERSE);

        /*gyro_sensor.calibrate();

        while (gyro_sensor.isCalibrating())
        {
            Thread.sleep(50);
        }*/

        waitForStart();

        while (right_drive.getCurrentPosition() < 12000)
        {
            right_drive.setPower(0.5);
            left_drive.setPower(0.5);
            telemetry.addData("Encoder Counts", right_drive.getCurrentPosition());
        }

        right_drive.setPower(0.0);
        left_drive.setPower(0.0);

        while (right_drive.getCurrentPosition() < 13440)
        {
            right_drive.setPower(0.5);
            left_drive.setPower(-0.5);
            // telemetry.addData("Gyro Heading", gyro_sensor.getHeading());
        }
    }
}
