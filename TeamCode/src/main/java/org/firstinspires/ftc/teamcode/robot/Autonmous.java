package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.subsystems.GreenLanternPipeline;
import org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import static org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage.carouselTask;
import static org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage.colorTask;
import static org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage.finalOptions;
import static org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage.parkTask;

@Config
@Autonomous(group = "main")
public class Autonmous extends LinearOpMode {

   GreenLanternPipeline pipeline;
   OpenCvWebcam webcam;

   Pose2d[] points = {};
   Trajectory[] trajectories = {};

   @Override
   public void runOpMode() throws InterruptedException {

       SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        pipeline = new GreenLanternPipeline();
        valueStorage poseStorage = new valueStorage();
        initPipeline();
       webcam.setPipeline(pipeline);

       while(!opModeIsActive())
        {
            telemetry.addData("Barcode Location:",pipeline.getLocation());
            telemetry.update();
        }

       if(isStopRequested())
       {
           webcam.stopStreaming();
       }

        points = listOfPose();

        for(int i = 0; i < points.length; i++)
        {
            Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                    .lineToSplineHeading(points[i])
                    .build();

            trajectories[i] = trajectory;
        }

        waitForStart();

        webcam.stopStreaming();

        drive.followTrajectory(trajectories[0]);
        // TODO: ADD CODE FOR THE TRAJECTORY(LIKE GET FREIGHT AND THAT)
        drive.followTrajectory(trajectories[1]);
        // TODO: ADD CODE FOR THE TRAJECTORY(LIKE GET FREIGHT AND THAT)
        drive.setMotorPowers(0,0,0,0);

        while(opModeIsActive())
        {
            telemetry.update();
            poseStorage.currentPose = drive.getPoseEstimate();
        }

    }

    public Pose2d[] listOfPose()
    {
        // TODO: REMOVE TELEMETRY

        int orderCarousel = 0, orderPark = 1;
        Pose2d[] pose2DS = {};
        boolean blue = false, red = false;

        if(colorTask[0].equals(finalOptions[0]))
        {
            blue = true;
            red = false;
        }
        else if(colorTask[1].equals(finalOptions[0]))
        {
            blue = false;
            red = true;
        }

        if(carouselTask[0].equals(finalOptions[1]))
        {
            if(blue)
            {
                telemetry.addLine("blue! carousel yes");
                pose2DS[orderCarousel] = new Pose2d(1,1, 0);
            }
            else
            {
                telemetry.addLine("red! carousel yes");
                pose2DS[orderCarousel] = new Pose2d(-1,-1, 0);
            }
        }
        else if(carouselTask[1].equals(finalOptions[1]))
        {
            if(blue)
            {
                telemetry.addLine("blue! carousel no");
                pose2DS[orderCarousel] = new Pose2d(2,2, 0);
            }
            else
            {
                telemetry.addLine("red! carousel no");
                pose2DS[orderCarousel] = new Pose2d(-2,-2, 0);
            }
        }

        if(parkTask[0].equals(finalOptions[2]))
        {
            if(blue)
            {
                telemetry.addLine("park blue! Not Completely In ASU");
                pose2DS[orderPark] = new Pose2d(1,1, 0);
            }
            else
            {
                telemetry.addLine("park red! Not Completely In ASU");
                pose2DS[orderPark] = new Pose2d(-1,-1, 0);
            }
        }
        else if(parkTask[1].equals(finalOptions[2]))
        {
            if(blue)
            {
                telemetry.addLine("park blue! Completely In ASU");
                pose2DS[orderPark] = new Pose2d(2,2, 0);
            }
            else
            {
                telemetry.addLine("park red! Completely In ASU");
                pose2DS[orderPark] = new Pose2d(-2,-2, 0);
            }
        }
        else if(parkTask[2].equals(finalOptions[2]))
        {
            if(blue)
            {
                telemetry.addLine("park blue! Not Completely In WH");
                pose2DS[orderPark] = new Pose2d(3,3, 0);
            }
            else
            {
                telemetry.addLine("park red! Not Completely In WH");
                pose2DS[orderPark] = new Pose2d(-3,-3, 0);
            }
        }
        else if(parkTask[3].equals(finalOptions[2]))
        {
            if(blue)
            {
                telemetry.addLine("park blue! Completely In WH");
                pose2DS[orderPark] = new Pose2d(4,4, 0);
            }
            else
            {
                telemetry.addLine("park red! Completely In WH");
                pose2DS[orderPark] =  new Pose2d(-4,-4, 0);
            }
        }

        return pose2DS;
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

}