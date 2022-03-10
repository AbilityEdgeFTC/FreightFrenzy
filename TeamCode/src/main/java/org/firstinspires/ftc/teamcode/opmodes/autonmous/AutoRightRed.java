package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "Right Red FULL", group = "red")
public class AutoRightRed extends LinearOpMode {

    double startPoseRightX = 13;
    double startPoseRightY = -60;
    double startPoseRightH = 90;
    public static double turnPoseRightX = 0;
    public static double turnPoseRightY = -62;
    public static double turnPoseRightH = 180;
    public static double poseEntranceX = 19.5;
    public static double poseEntranceY = -63;
    public static double poseEntranceH = 180;
    public static double poseCollectX = 63;
    public static double poseCollectY = -63;
    public static double poseCollectH = 180;

    //carousel carousel;
    intake intake;
    //dip dip;
    //ElevatorThreadAuto threadAuto;
    //public static double reverseIntakeFor = .8;
    //OpenCvWebcam webcam;
    //YCbCrPipeline pipeline;
    SampleMecanumDrive drive;

    TrajectorySequence main;

    //public static boolean withVision = true;

    /*enum levels
    {
        MIN,
        MID,
        MAX
    }

    levels placeFreightIn = levels.MAX;*/

    @Override
    public void runOpMode() throws InterruptedException {
        //pipeline = new YCbCrPipeline();
        //pipeline.telemetry = telemetry;
        //pipeline.DEBUG = false;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d turnPoseRight = new Pose2d(turnPoseRightX,turnPoseRightY,Math.toRadians(turnPoseRightH));
        Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));

        intake = new intake(hardwareMap);
        //initPipeline();
        //webcam.setPipeline(pipeline);

        drive.setPoseEstimate(startPoseRight);

        main = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(5)
                .lineToSplineHeading(poseEntrance,
                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    intake.intakeForward();
                })
                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                .addDisplacementMarker(pathLength -> pathLength * 0.45, () -> {
                    intake.intakeBackward();
                })
                .lineToSplineHeading(poseEntrance)
                .addDisplacementMarker(() -> {
                    intake.intakeForward();
                })
                .waitSeconds(2)
                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                .addDisplacementMarker(pathLength -> pathLength * 0.45, () -> {
                    intake.intakeBackward();
                })
                .lineToSplineHeading(poseEntrance)
                .addDisplacementMarker(() -> {
                    intake.intakeForward();
                })
                .waitSeconds(2)
                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                .addDisplacementMarker(pathLength -> pathLength * 0.45, () -> {
                    intake.intakeBackward();
                })
                .lineToSplineHeading(poseEntrance)
                .addDisplacementMarker(() -> {
                    intake.intakeForward();
                })
                .waitSeconds(2)
                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                .addDisplacementMarker(pathLength -> pathLength * 0.45, () -> {
                    intake.intakeBackward();
                })
                .lineToSplineHeading(poseEntrance)
                .addDisplacementMarker(() -> {
                    intake.intakeForward();
                })
                .waitSeconds(2)
                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                .addDisplacementMarker(pathLength -> pathLength * 0.45, () -> {
                    intake.intakeBackward();
                })
                .lineToSplineHeading(poseEntrance)
                .addDisplacementMarker(() -> {
                    intake.intakeForward();
                })
                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                .build();

        //threadAuto.start();
        //dip.getFreight();

        /*while (!opModeIsActive() && !isStopRequested())
        {
            switch (pipeline.getLocation())
            {
                case Left:
                    //IF BARCODE IS ON LEFT SIDE
                    placeFreightIn = levels.MIN;
                    break;
                case Center:
                    //IF BARCODE IS ON CENTER SIDE
                    placeFreightIn = levels.MID;
                    break;
                case Right:
                    //IF BARCODE IS ON RIGHT SIDE
                    placeFreightIn = levels.MAX;
                    break;
                default:
                    placeFreightIn = levels.MAX;
                    break;
            }
            telemetry.addData("Barcode Location:", pipeline.getLocation());
            telemetry.update();
        }*/

        waitForStart();

        /*if(withVision){
            webcam.stopStreaming();
        }*/

        drive.followTrajectorySequence(main);
    }

//    void initPipeline()
//    {
//        //setting up webcam from config, and displaying it in the teleop controller.
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//
//        //getting the pipeline and giving it telemetry. and setting the pipeline to the webcam
//        YCbCrPipeline pipeline = new YCbCrPipeline();
//        pipeline.telemetry = telemetry;
//        pipeline.DEBUG = false;
//
//        webcam.setPipeline(pipeline);
//
//        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                /*
//                 * Tell the webcam to start streaming images to us! Note that you must make sure
//                 * the resolution you specify is supported by the camera. If it is not, an exception
//                 * will be thrown.
//                 *
//                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
//                 * supports streaming from the webcam in the uncompressed YUV image format. This means
//                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
//                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
//                 *
//                 * Also, we specify the rotation that the webcam is used in. This is so that the image
//                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
//                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
//                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
//                 * away from the user.
//                 */
//                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
//                FtcDashboard.getInstance().startCameraStream(webcam,0);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
//    }

}
