/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.risky;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.risky.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

import java.util.ArrayList;

@Autonomous(name = "left risky auto")
public class StangaRiskyAutonomie extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    AprilTagDetection tagOfInterest = null;
    DcMotorEx stSus;
    DcMotorEx stJos;
    DcMotorEx drSus;
    DcMotorEx drJos;


    @Override
    public void runOpMode()
    {
        stSus = hardwareMap.get(DcMotorEx.class, "stSus");
        drSus = hardwareMap.get(DcMotorEx.class, "drSus");
        stJos = hardwareMap.get(DcMotorEx.class, "stJos");
        drJos = hardwareMap.get(DcMotorEx.class, "drJos");
        stSus.setDirection(DcMotorSimple.Direction.REVERSE);
        stJos.setDirection(DcMotorSimple.Direction.FORWARD);
        drJos.setDirection(DcMotorSimple.Direction.FORWARD);
        drSus.setDirection(DcMotorSimple.Direction.REVERSE);
        stSus.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stJos.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drSus.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drJos.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stSus.setPower(0);
        stJos.setPower(0);
        drSus.setPower(0);
        drJos.setPower(0);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.FRONT, cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;

                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        waitForStart();

        int id = 3;
        if(tagOfInterest != null)
            id = tagOfInterest.id;
        if(id == 1) {
            zone1(1000);
        }
        if(id == 2) {
            zone2(1000);
        }
        if(id == 3) {
            zone3(1000);
        }
        telemetry.addData("id", id);
        telemetry.update();
    }
    void zone2(long mili)
    {

        stSus.setPower(.5);
        drSus.setPower(.5);
        stJos.setPower(.5);
        drJos.setPower(.5);
        sleep(mili);
        stSus.setPower(0);
        drSus.setPower(0);
        stJos.setPower(0);
        drJos.setPower(0);
    }
    void zone1(long mili)
    {
        stSus.setPower(-0.5);
        drSus.setPower(0.5);
        stJos.setPower(0.5);
        drJos.setPower(-0.5);
        sleep(mili);
        stSus.setPower(.5);
        drSus.setPower(.5);
        stJos.setPower(.5);
        drJos.setPower(.5);
        sleep(mili);
        stSus.setPower(0);
        drSus.setPower(0);
        stJos.setPower(0);
        drJos.setPower(0);
    }
    void zone3(long mili)
    {
        stSus.setPower(-0.5);
        drSus.setPower(0.5);
        stJos.setPower(0.5);
        drJos.setPower(-0.5);
        sleep(mili);

        sleep(250);
        stSus.setPower(.5);
        drSus.setPower(-.5);
        stJos.setPower(-.5);
        drJos.setPower(.5);
        sleep(mili*2);
        stSus.setPower(.5);
        drSus.setPower(.5);
        stJos.setPower(.5);
        drJos.setPower(.5);
        sleep(mili);
        stSus.setPower(0);
        drSus.setPower(0);
        stJos.setPower(0);
        drJos.setPower(0);
    }
    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}