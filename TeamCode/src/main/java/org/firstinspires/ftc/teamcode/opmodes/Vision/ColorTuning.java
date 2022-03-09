/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmodes.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@TeleOp(group="Tests")
public class ColorTuning extends LinearOpMode {

    OpenCvWebcam webcam;
    cGamepad cGamepad1;
    cGamepad cGamepad2;
    public static boolean TSE = true;

    public static double hMax = 255;
    public static double sMax = 255;
    public static double vMax = 255;

    public static double hMin = 0;
    public static double sMin = 0;
    public static double vMin = 0;

    MenuPipeline pipeline;

    public static boolean isMax = false;

    @Override
    public void runOpMode() {
        //getting the pipeline and giving it telemetry. and setting the pipeline to the webcam

        //getting the pipeline and giving it telemetry. and setting the pipeline to the webcam
        pipeline = new MenuPipeline();
        double[] max ={hMax,sMax,vMax};
        double[] min ={hMin,sMin,vMin};

        int curentType =0;

        initPipeline();
        cGamepad1 = new cGamepad(gamepad1);
        cGamepad2 = new cGamepad(gamepad2);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            cGamepad1.update();

            if (cGamepad1.XOnce())
            {
                isMax = false;
            }
            if (cGamepad1.BOnce())
            {
                isMax = true;
            }

            /**min or max **/
            //max
            if(isMax) {
                /**the type changed **/
                //down
                if (cGamepad2.dpadDownOnce() && curentType > 0) {
                    curentType--;
                }
                if (cGamepad2.dpadUpOnce() && curentType<max.length -1) {
                    curentType++;
                }
                //up
                /**the value change byu 1 **/
                //-1
                if (cGamepad2.dpadLeftOnce() && max[curentType]>1) {
                    max[curentType]--;
                }
                //+1
                if (cGamepad2.dpadRightOnce() && max[curentType]<254) {
                    max[curentType]++;
                }
                /**the value change byu 10 **/
                //-10
                if (cGamepad2.leftBumperOnce() && max[curentType]>10) {
                    max[curentType] -= 10;
                }
                //+10
                if (cGamepad2.rightBumperOnce() && max[curentType]<245) {
                    max[curentType] += 10;
                }

            }
            //min
            else {
                /**the type changed **/
                //down
                if (cGamepad1.dpadDownOnce() && curentType > 0) {
                    curentType--;
                }
                if (cGamepad1.dpadUpOnce() && curentType < min.length-1) {
                    curentType++;
                }
                //up
                /**the value change byu 1 **/
                //-1
                if (cGamepad1.dpadLeftOnce() && min[curentType] > 1) {
                    min[curentType]--;
                }
                //+1
                if (cGamepad1.dpadRightOnce() && min[curentType] < 254) {
                    min[curentType]++;
                }
                /**the value change byu 10 **/
                //-10
                if (cGamepad1.leftBumperOnce() && min[curentType] > 10) {
                    min[curentType] -= 10;
                }
                //+10
                if (cGamepad1.rightBumperOnce() && min[curentType] < 245) {
                    min[curentType] += 10;
                }

            }

            telemetry.addLine("(h) " +  ": " +min[0] + " " + max[0]);
            telemetry.addLine("(s) " +  ": " +min[1] + " " + max[1]);
            telemetry.addLine("(v) " +  ": " +min[2] + " " + max[2]);
            telemetry.addData("LEFT", pipeline.barcodeLeft);
            telemetry.addData("CENTER", pipeline.barcodeCenter);
            telemetry.addData("RIGHT", pipeline.barcodeRight);
            //}

//            if(cGamepad1.AOnce())
//            {
//                pipeline.setMax(max);
//                pipeline.setMin(min);
//            }
            //telemetry.addData("Barcode Location:",pipeline.getLocation());
            telemetry.update();
        }

        //STOP streaming when opmode is done.
        webcam.stopStreaming();
    }

    public void initPipeline()
    {
        //setting up webcam from config, and displaying it in the teleop controller.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //getting the pipeline and giving it telemetry. and setting the pipeline to the webcam
        MenuPipeline pipeline = new MenuPipeline();
        pipeline.telemetry = telemetry;
        pipeline.DEBUG = false;

        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam,0);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }
}
