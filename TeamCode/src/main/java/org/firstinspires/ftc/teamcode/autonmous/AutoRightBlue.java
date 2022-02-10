package org.firstinspires.ftc.teamcode.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.MecanumLocalizer;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorThreadAuto;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorThreadAuto.ElevatorState;
import org.firstinspires.ftc.teamcode.robot.subsystems.GreenLanternPipeline;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "Right Blue FULL", group = "blue")
public class AutoRightBlue extends LinearOpMode {

    public static double startPoseLeftX = -33.6;
    public static double startPoseLeftY = 64.04;
    public static double startPoseLeftH = 180;
    public static double poseCarouselX = -59.5;
    public static double poseCarouselY = 57.5;
    public static double poseCarouselH = 45;
    public static double carouselHelp = 15;
    public static double poseHubLeftX = -32;
    public static double poseHubLeftY = 21;
    public static double poseHubLeftH = 0;
    public static double parkBack = 28;
    public static double parkRight = 12;
    public static double runCarouselFor = 10;
    carousel carousel;
    intake intake;
    dip dip;
    ElevatorThreadAuto threadAuto;
    OpenCvWebcam webcam;
    GreenLanternPipeline pipeline;
    SampleMecanumDriveCancelable drive;

    TrajectorySequence carouselGo,hub,parking;

    public static boolean withVision = true;

    enum levels
    {
        MIN,
        MID,
        MAX
    }

    levels placeFreightIn = levels.MAX;

    @Override
    public void runOpMode() throws InterruptedException {
        pipeline = new GreenLanternPipeline();
        pipeline.telemetry = telemetry;
        pipeline.DEBUG = false;
        pipeline.TSE = true;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDriveCancelable(hardwareMap);

        Pose2d startPoseLeft = new Pose2d(startPoseLeftX, startPoseLeftY, Math.toRadians(startPoseLeftH));
        Pose2d poseCarousel = new Pose2d(poseCarouselX, poseCarouselY, Math.toRadians(poseCarouselH));
        Pose2d poseHubLeft = new Pose2d(poseHubLeftX, poseHubLeftY, Math.toRadians(poseHubLeftH));

        carousel = new carousel(hardwareMap);
        intake = new intake(hardwareMap);
        dip = new dip(hardwareMap);
        threadAuto = new ElevatorThreadAuto(hardwareMap);

        initPipeline();
        webcam.setPipeline(pipeline);

        drive.setPoseEstimate(startPoseLeft);

        carouselGo = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeLeft(carouselHelp)
                .lineToLinearHeading(poseCarousel)
                .build();

        hub = drive.trajectorySequenceBuilder(carouselGo.end())
                .lineToSplineHeading(poseHubLeft)
                .build();

        parking = drive.trajectorySequenceBuilder(hub.end())
                .back(parkBack)
                .strafeLeft(parkRight)
                .build();

        threadAuto.start();
        dip.getFreight();

        while (!opModeIsActive())
        {
            switch (pipeline.getLocation())
            {
                case Left:
                    //IF BARCODE IS ON LEFT SIDE
                    placeFreightIn = levels.MAX;
                    break;
                case Center:
                    //IF BARCODE IS ON CENTER SIDE
                    placeFreightIn = levels.MID;
                    break;
                case Right:
                    //IF BARCODE IS ON RIGHT SIDE
                    placeFreightIn = levels.MIN;
                    break;
                default:
                    placeFreightIn = levels.MAX;
                    break;
            }
            telemetry.addData("Barcode Location:", pipeline.getLocation());
            telemetry.update();
        }

        waitForStart();

        if(withVision){
            webcam.stopStreaming();
        }

        drive.followTrajectorySequence(carouselGo);
        runCarousel();
        drive.followTrajectorySequence(hub);
        switch (placeFreightIn)
        {
            case MIN:
                goToMin();
                break;
            case MID:
                goToMid();
                break;
            case MAX:
                goToMax();
                break;
        }
        drive.followTrajectorySequence(parking);
        if(isStopRequested())
        {
            threadAuto.interrupt();
            drive.breakFollowing();
            drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.setMotorPowers(0, 0, 0, 0);
        }
    }

    void goToMin() throws InterruptedException
    {
        if(opModeIsActive())
        {
            threadAuto.setElevatorState(ElevatorThreadAuto.ElevatorState.MIN);
            sleep(800);
            dip.releaseFreightPos();
            sleep(1000);
            dip.releaseFreight();
            dip.getFreight();
            threadAuto.setElevatorState(ElevatorState.ZERO);
        }
    }

    void goToMid() throws InterruptedException
    {
        if(opModeIsActive())
        {
            threadAuto.setElevatorState(ElevatorThreadAuto.ElevatorState.MID);
            sleep(800);
            dip.releaseFreightPos();
            sleep(1000);
            dip.releaseFreight();
            dip.getFreight();
            threadAuto.setElevatorState(ElevatorState.ZERO);
        }
    }

    void goToMax() throws InterruptedException
    {
        if(opModeIsActive())
        {
            threadAuto.setElevatorState(ElevatorThreadAuto.ElevatorState.MAX);
            sleep(800);
            dip.releaseFreightPos();
            sleep(1000);
            dip.releaseFreight();
            dip.getFreight();
            threadAuto.setElevatorState(ElevatorState.ZERO);
        }
    }

    void initPipeline()
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

    void runCarousel() throws InterruptedException {
        carousel.spin(false, false);
        Thread.sleep((long)(runCarouselFor * 1000));
        carousel.stop();
    }
}
