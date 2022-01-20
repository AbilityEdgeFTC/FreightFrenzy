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
import org.firstinspires.ftc.teamcode.robot.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.subsystems.GreenLanternPipeline;
import org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(group = "main")
public class Autonmous extends LinearOpMode {

    public static String taskName = "";
    public static String[][] tasks = {};
    public static String[] autoTask = {};
    public static String[] allianceTask = {};
    public static String[] startPosTask = {};
    public static String[] startDelayTask = {};
    public static String[] carouselTask = {};
    public static String[] collectFreightTask = {};
    public static String[] numOfFreightTask = {};
    public static String[] placeFreightAtTask = {};
    public static String[] parkInTask = {};
    public static String[] parkTypeTask = {};
    public static String[] finalOptions = {};
    public static int[] currentOption = {};

   GreenLanternPipeline pipeline;
   OpenCvWebcam webcam;

   Pose2d[] points = {};
   Trajectory[] trajectories = {};

   boolean runAuto = true;

   @Override
   public void runOpMode() throws InterruptedException {

       SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        pipeline = new GreenLanternPipeline();
        valueStorage poseStorage = new valueStorage();
        initPipeline();
       initOptions();
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

        if(runAuto)
        {
            drive.followTrajectory(trajectories[0]);
            // TODO: ADD CODE FOR THE TRAJECTORY(LIKE GET FREIGHT AND THAT)
            drive.followTrajectory(trajectories[1]);
            // TODO: ADD CODE FOR THE TRAJECTORY(LIKE GET FREIGHT AND THAT)
            drive.setMotorPowers(0, 0, 0, 0);
        }

        while(opModeIsActive())
        {
            telemetry.update();
            poseStorage.currentPose = drive.getPoseEstimate();
        }

    }
    void initOptions()
    {
        autoTask[0] = "YES";
        autoTask[1] = "NO";

        allianceTask[0] = "Red";
        allianceTask[1] = "Blue";

        startPosTask[0] = "Right";
        startPosTask[1] = "Left";

        for(int i = 0; i < 30; i++)
        {
            startDelayTask[i] = "" + i+1;
        }

        carouselTask[0] = "YES";
        carouselTask[1] = "NO";

        collectFreightTask[0] = "YES";
        collectFreightTask[1] = "NO";

        for(int i = 0; i < 10; i++)
        {
            numOfFreightTask[i] = "" + i+1;
        }

        placeFreightAtTask[0] = "LEFT";
        placeFreightAtTask[1] = "RIGHT";
        placeFreightAtTask[2] = "FRONT";
        placeFreightAtTask[3] = "BACK";

        parkInTask[0] = "Alliance Shipping Unit";
        parkInTask[1] = "Warehouse";

        parkTypeTask[0] = "Completely";
        parkTypeTask[0] = "Not Completely";

    }

    public Pose2d[] listOfPose()
    {
        finalOptions[0] = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("autoTask.txt"));;
        finalOptions[1] = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("allianceTask.txt"));
        finalOptions[2] = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("autoTask.txt"));
        finalOptions[3] = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("startPosTask.txt"));
        finalOptions[4] = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("startDelayTask.txt"));
        finalOptions[5] = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("carouselTask.txt"));
        finalOptions[6] = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("collectFreightTask.txt"));
        finalOptions[7] = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("numOfFreightTask.txt"));
        finalOptions[8] = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("parkInTask.txt"));
        finalOptions[9] = ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("parkTypeTask.txt"));

        /*

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
        }*/

        //return pose2DS;
        return null;
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