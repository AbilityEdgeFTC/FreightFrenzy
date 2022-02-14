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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.subsystems.GreenLanternPipeline;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@TeleOp(group="Tests")
public class ColorTuning extends LinearOpMode {

    OpenCvWebcam webcam;
    cGamepad cGamepad1;
    public static boolean TSE = true;

    static VisionObject tseH = new VisionObject(255,0,"H");
    static VisionObject tseS = new VisionObject(255,0,"S");
    static VisionObject tseV = new VisionObject(255,0,"V");

    Mat mask = new Mat(1280,720,0);//
    Mat inputHSV = new Mat(1280,720,0);

    public static VisionObject[] typeTuning ={tseH,tseS,tseV};


    @Override
    public void runOpMode() {
        //getting the pipeline and giving it telemetry. and setting the pipeline to the webcam
        GreenLanternPipeline pipeline = new GreenLanternPipeline();
        pipeline.telemetry = telemetry;
        pipeline.DEBUG = false;
        pipeline.TSE = true;

        int curentType =0;

        initPipeline();
        webcam.setPipeline(pipeline);
        cGamepad1 = new cGamepad(gamepad1);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /**min or max **/
            //max
            if (cGamepad1.BOnce())
            {
                /**the type changed **/
                //down
                if(cGamepad1.dpadDownOnce() && curentType > 0)
                {
                    curentType -=1;
                }
                else if(cGamepad1.dpadUpOnce() && curentType<typeTuning.length)
                {
                    curentType +=1;
                }
                //up
                /**the value change byu 1 **/
                //-1
                if (cGamepad1.dpadLeftOnce() && typeTuning[curentType].getHighValueTSE()>1)
                {
                    typeTuning[curentType].setHighValueTSE(typeTuning[curentType].getHighValueTSE()-1);
                }
                //+1
                else if(cGamepad1.dpadRightOnce() && typeTuning[curentType].getHighValueTSE()<254)
                {
                    typeTuning[curentType].setHighValueTSE(typeTuning[curentType].getHighValueTSE()+1);
                }
                /**the value change byu 10 **/
                //-10
                if(cGamepad1.leftBumperOnce() && typeTuning[curentType].getHighValueTSE()>10)
                {
                    typeTuning[curentType].setHighValueTSE(typeTuning[curentType].getHighValueTSE()-10);
                }
                //+10
                else if(cGamepad1.rightBumperOnce() && typeTuning[curentType].getHighValueTSE()<245)
                {
                    typeTuning[curentType].setHighValueTSE(typeTuning[curentType].getHighValueTSE()+10);
                }
            }
            //min
            else if(cGamepad1.XOnce())
            {
                /**the type changed **/
                //down
                if(cGamepad1.dpadDownOnce() && curentType > 0)
                {
                    curentType -=1;
                }
                else if(cGamepad1.dpadUpOnce() && curentType<typeTuning.length)
                {
                    curentType +=1;
                }
                //up
                /**the value change byu 1 **/
                //-1
                if (cGamepad1.dpadLeftOnce() && typeTuning[curentType].getLowValueTSE()>1)
                {
                    typeTuning[curentType].setLowValueTSE(typeTuning[curentType].getLowValueTSE()-1);
                }
                //+1
                else if(cGamepad1.dpadRightOnce() && typeTuning[curentType].getLowValueTSE()<254)
                {
                    typeTuning[curentType].setLowValueTSE(typeTuning[curentType].getLowValueTSE()+1);
                }
                /**the value change byu 10 **/
                //-10
                if(cGamepad1.leftBumperOnce() && typeTuning[curentType].getLowValueTSE()>10)
                {
                    typeTuning[curentType].setLowValueTSE(typeTuning[curentType].getLowValueTSE()-10);
                }
                //+10
                else if(cGamepad1.rightBumperOnce() && typeTuning[curentType].getLowValueTSE()<245)
                {
                    typeTuning[curentType].setLowValueTSE(typeTuning[curentType].getLowValueTSE()+10);
                }
            }


            telemetry.addData("Barcode Location:",pipeline.getLocation());
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
        GreenLanternPipeline pipeline = new GreenLanternPipeline();
        pipeline.telemetry = telemetry;
        pipeline.DEBUG = false;
        pipeline.TSE = true;

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
    public Mat processFrame(Mat input)  {
        // HSV low and high values for our team shipping element.
        Scalar lowValuesTSE = new Scalar(tseH.getLowValueTSE(), tseS.getLowValueTSE(), tseV.getLowValueTSE());
        Scalar highValuesTSE = new Scalar(tseH.getHighValueTSE(), tseS.getHighValueTSE(), tseV.getHighValueTSE());
        Imgproc.cvtColor(input, inputHSV, Imgproc.COLOR_RGB2HSV);



        Imgproc.cvtColor(input, inputHSV, Imgproc.COLOR_RGB2HSV);

        // turning all colors not between the low and high values to black and the rest white.
        if (TSE) {
            Core.inRange(inputHSV, lowValuesTSE, highValuesTSE, mask);
            telemetry.addLine("Using: TSE");
        } else {
            Core.inRange(inputHSV, lowValuesTSE, highValuesTSE, mask);
            telemetry.addLine("Using: NONE");
        }
        Imgproc.cvtColor(mask, mask, Imgproc.COLOR_GRAY2RGB);
        return mask;
    }
}
