/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.ftcrobotcontroller.autonomouslibs.RobotStates;
import org.lasarobotics.vision.test.android.Cameras;
import org.lasarobotics.vision.test.detection.ColorBlobDetector;
import org.lasarobotics.vision.test.detection.objects.Contour;
import org.lasarobotics.vision.test.ftc.resq.Beacon;
import org.lasarobotics.vision.test.image.Drawing;
import org.lasarobotics.vision.test.opmode.ManualVisionOpMode;
import org.lasarobotics.vision.test.util.color.ColorHSV;
import org.lasarobotics.vision.test.util.color.ColorRGBA;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Size;

import java.util.List;

/**
 * TeleOp Mode
 * <p/>
 * Enables control of the robot via the gamepad
 */
public class ManualVisionSample extends ManualVisionOpMode {

    private static final ColorHSV lowerBoundRed = new ColorHSV((int) (305 / 360.0 * 255.0), (int) (0.200 * 255.0), (int) (0.300 * 255.0));
    private static final ColorHSV upperBoundRed = new ColorHSV((int) ((360.0 + 5.0) / 360.0 * 255.0), 255, 255);
    private static final ColorHSV lowerBoundBlue = new ColorHSV((int) (170.0 / 360.0 * 255.0), (int) (0.200 * 255.0), (int) (0.750 * 255.0));
    private static final ColorHSV upperBoundBlue = new ColorHSV((int) (227.0 / 360.0 * 255.0), 255, 255);
    private Beacon.BeaconAnalysis colorAnalysis = new Beacon.BeaconAnalysis();
    private ColorBlobDetector detectorRed;
    private ColorBlobDetector detectorBlue;
    private boolean noError = true;
    private double beaconWidth = 21.8;
    private double beaconHeight = 14.5;
    public double beaconCenterX;
    public double beaconCenterY;
    public String beaconColor = new String();
    public String analyConfi = new String();
    private int widthFrame = 910;
    private int heightFrame = 864;
    private GyroSensor gyroSensor;
    private static RobotStates robotState;
    //DcMotor debris_motor;
    DcMotor right_drive;
    DcMotor left_drive;
    DcMotor tread_drive;
    @Override
    public void init() {
        super.init();
        robotState = RobotStates.LOOKING_FOR_BEACON;
        //Initialize all detectors here
        detectorRed = new ColorBlobDetector(lowerBoundRed, upperBoundRed);
        detectorBlue = new ColorBlobDetector(lowerBoundBlue, upperBoundBlue);

        this.setCamera(Cameras.PRIMARY);
        this.setFrameSize(new Size(900, 900));

        // Initialize motors

        //debris_motor = hardwareMap.dcMotor.get("debris_motor");
        right_drive = hardwareMap.dcMotor.get("right_drive");
        left_drive = hardwareMap.dcMotor.get("left_drive");
        tread_drive = hardwareMap.dcMotor.get("tread_drive");
//        linear_slide = hardwareMap.dcMotor.get("linear_slide");
//        vertical_pivot = hardwareMap.dcMotor.get("vertical_pivot");
        gyroSensor = hardwareMap.gyroSensor.get("gyro_sensor");

    }

    @Override
    public void loop() {
        super.loop();
        telemetry.addData("Vision FPS", fps.getFPSString());
        telemetry.addData("Vision Color", colorAnalysis.toString());
        telemetry.addData("Analysis Confidence", colorAnalysis.getConfidence());
        telemetry.addData("Vision Size", "Width: " + width + " Height: " + height);
        telemetry.addData("Vision Status", noError ? "OK!" : "ANALYSIS ERROR!");
        telemetry.addData("RobotState:", robotState);
        stateMachine();
    }

    public void stateMachine() {
        telemetry.addData("State Machine Entered", robotState);
        switch (robotState) {
            case LOOKING_FOR_BEACON:
                left_drive.setPower(-0.15);
                right_drive.setPower(-0.15);
                if (colorAnalysis.getConfidence() > 0.0) {
                    robotState = RobotStates.BEACON_FOUND;
                }
                break;
            case BEACON_FOUND:
                telemetry.addData("Beacon Found, Confidence: ", colorAnalysis.getConfidenceString());
                left_drive.setPower(0.1);
                right_drive.setPower(0.1);
                telemetry.addData("Beacon Center X", beaconCenterX);
                if(beaconCenterX <= ((widthFrame/2) + 30) && beaconCenterX >= ((widthFrame/2) - 30)) {
                    //robotState = RobotStates.CENTERED_ON_BEACON;
                }
                break;
        }

    }

    @Override
     public void stop() {
        super.stop();
    }

    @Override
    public Mat frame(Mat rgba, Mat gray) {
        try {
            //Process the frame for the color blobs
            detectorRed.process(rgba);
            detectorBlue.process(rgba);

            //Get the list of contours
            List<Contour> contoursRed = detectorRed.getContours();
            List<Contour> contoursBlue = detectorBlue.getContours();

            //Get color analysis
            Beacon beacon = new Beacon();
            beaconCenterX = beacon.getBeaconCenter().x;
            beaconCenterY = beacon.getBeaconCenter().y;
            beaconColor = colorAnalysis.toString();
            analyConfi = colorAnalysis.getConfidenceString();
            colorAnalysis = beacon.analyzeColor(contoursRed, contoursBlue, rgba, gray);
            telemetry.addData("Beacon Center X", beacon.getBeaconCenter().x);
            telemetry.addData("Beacon Center Y", beacon.getBeaconCenter().y);
            telemetry.addData("Beacon Size X", beacon.getBeaconSize().width);
            telemetry.addData("Beacon Size Y", beacon.getBeaconSize().height);
//            Matrix beaconSize = new Matrix();
//            beaconSize.setValues();
            telemetry.addData("Beacon Actual Size X", beaconWidth);
            telemetry.addData("Beacon Actual Size Y", beaconHeight);

            //Enable to Debug Camera
            //Draw red and blue contours
            //Drawing.drawContours(rgba, contoursRed, new ColorRGBA(255, 0, 0), 2);
            //Drawing.drawContours(rgba, contoursBlue, new ColorRGBA(0, 0, 255), 2);

            //Transform.enlarge(mRgba, originalSize, true);
            //Transform.enlarge(mGray, originalSize, true);

            //Enable to Debug Camera
            //Draw text
            //Drawing.drawText(rgba, colorAnalysis.toString(),
            //        new Point(0, 8), 1.0f, new ColorGRAY(255), Drawing.Anchor.BOTTOMLEFT);

            noError = true;
        }
        catch (Exception e)
        {
            Drawing.drawText(rgba, "Analysis Error", new Point(0, 8), 1.0f, new ColorRGBA("#F44336"), Drawing.Anchor.BOTTOMLEFT);
            noError = false;
            e.printStackTrace();
        }

        //Enable to Debug Camera
        //Drawing.drawText(rgba, "FPS: " + fps.getFPSString(), new Point(0, 24), 1.0f, new ColorRGBA("#ffffff")); //"#2196F3"

//        Matrix camMatrix = new Matrix();
//        if (rgba.isContinuous()) {
//            camMatrix.setValues();
//        } else {
//            float[] opencv_matrix_values = new float[9];
//            // Undocumented .get(row, col, float[]), but seems to be bulk-copy.
//            rgba.get(0, 0, opencv_matrix_values);
//            camMatrix.setValues(opencv_matrix_values);
//        }
//        ImageView cvCameraView = new ImageView();
//        cvCameraView.setImageMatrix(rgba);


        return rgba;
    }
}
