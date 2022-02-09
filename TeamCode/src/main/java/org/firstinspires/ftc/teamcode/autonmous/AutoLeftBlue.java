package org.firstinspires.ftc.teamcode.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.MecanumLocalizer;
import org.firstinspires.ftc.teamcode.robot.subsystems.GreenLanternPipeline;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorThreadAuto;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorThreadAuto.ElevatorState;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class AutoLeftBlue extends LinearOpMode {

    public static double startPoseRightX = 12;
    public static double startPoseRightY = 64.24;
    public static double startPoseRightH = 0;
    public static double poseHubFrontX = -12;
    public static double poseHubFrontY = 42;
    public static double poseHubFrontH = 270;
    public static double poseEntranceX = 12;
    public static double poseEntranceY = 64;
    public static double poseEntranceH = 180;
    public static double poseCollectX = 60;
    public static double poseCollectY = 64;
    public static double poseCollectH = 180;
    public static double entranceHelp = 2;
    carousel carousel;
    intake intake;
    dip dip;
    ElevatorThreadAuto threadAuto;
    public static double reverseIntakeFor = .8;
    OpenCvWebcam webcam;

    enum levels
    {
        MIN,
        MID,
        MAX
    }

    levels placeFreightIn = levels.MAX;

    //TrajectorySequence collect, placement, collect2, collect3, entrance, entrance2, entrance3, cycle, cycle2;
    @Override
    public void runOpMode() throws InterruptedException {
        GreenLanternPipeline pipeline = new GreenLanternPipeline();
        pipeline.telemetry = telemetry;
        pipeline.DEBUG = false;
        pipeline.TSE = true;
        pipeline.startingFromRight = false;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumLocalizer drive = new MecanumLocalizer(hardwareMap);

        Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        Pose2d poseHubFront = new Pose2d(poseHubFrontX, poseHubFrontY, Math.toRadians(poseHubFrontH));
        Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));

        carousel = new carousel(hardwareMap);
        intake = new intake(hardwareMap);
        dip = new dip(hardwareMap);
        threadAuto = new ElevatorThreadAuto(hardwareMap);

        drive.setPoseEstimate(startPoseRight);

        initPipeline();
        webcam.setPipeline(pipeline);

        //, MecanumLocalizer.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
        //                        MecanumLocalizer.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

        TrajectorySequence placement = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(poseHubFront)
                .build();

        TrajectorySequence entrance = drive.trajectorySequenceBuilder(placement.end())
                .lineToLinearHeading(poseEntrance)
                .build();

        TrajectorySequence collect = drive.trajectorySequenceBuilder(entrance.end())
                .lineToLinearHeading(new Pose2d(poseCollect.getX()-15,poseCollect.getY(),poseCollect.getHeading() - Math.toRadians(1.5)))
                .lineToLinearHeading(new Pose2d(poseCollect.getX()-10,poseCollect.getY(),poseCollect.getHeading() + Math.toRadians(1.5)))
                .build();

        TrajectorySequence cycle = drive.trajectorySequenceBuilder(collect.end())
                .lineToLinearHeading(poseEntrance)
                .lineToLinearHeading(poseHubFront)
                .build();

        TrajectorySequence entrance2 = drive.trajectorySequenceBuilder(cycle.end())
                .lineToLinearHeading(poseEntrance)
                .build();

        TrajectorySequence collect2 = drive.trajectorySequenceBuilder(entrance2.end())
                .lineToLinearHeading(new Pose2d(poseCollect.getX()-10,poseCollect.getY()+2,poseCollect.getHeading() - Math.toRadians(2)))
                .lineToLinearHeading(new Pose2d(poseCollect.getX()-5,poseCollect.getY()+2,poseCollect.getHeading() + Math.toRadians(2)))
                .build();

        TrajectorySequence cycle2 = drive.trajectorySequenceBuilder(collect2.end())
                .lineToLinearHeading(poseEntrance)
                .lineToLinearHeading(poseHubFront)
                .build();

        TrajectorySequence entrance3 = drive.trajectorySequenceBuilder(cycle2.end())
                .lineToLinearHeading(poseEntrance)
                .strafeRight(entranceHelp)
                .build();

        TrajectorySequence collect3 = drive.trajectorySequenceBuilder(entrance3.end())
                .lineToLinearHeading(new Pose2d(poseCollect.getX()-5,poseCollect.getY(),poseCollect.getHeading() - Math.toRadians(2.5)))
                .lineToLinearHeading(new Pose2d(poseCollect.getX()-2,poseCollect.getY(),poseCollect.getHeading() + Math.toRadians(2.5)))
                .build();

        threadAuto.start();
        dip.getFreight();

        while (!opModeIsActive()) {

            switch (pipeline.getLocation()){
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

            telemetry.addData("Barcode Location:",pipeline.getLocation());
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested())  threadAuto.interrupt();

        webcam.stopStreaming();
        drive.followTrajectorySequence(placement);
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
        drive.followTrajectorySequence(entrance);
        intake.intakeForward();
        drive.followTrajectorySequence(collect);
        Thread.sleep(1500);
        fixIntake();
        drive.followTrajectorySequence(cycle);
        goToMax();
        drive.followTrajectorySequence(entrance2);
        intake.intakeForward();
        drive.followTrajectorySequence(collect2);
        Thread.sleep(1500);
        fixIntake();
        drive.followTrajectorySequence(cycle2);
        goToMax();
        drive.followTrajectorySequence(entrance3);
        intake.intakeForward();
        drive.followTrajectorySequence(collect3);
        Thread.sleep(1500);
        fixIntake();
        threadAuto.interrupt();

        while (opModeIsActive())
        {
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();
        }
        threadAuto.interrupt();

    }

    void goToMin() throws InterruptedException {
        threadAuto.setElevatorState(ElevatorThreadAuto.ElevatorState.MIN);
        Thread.sleep(800);
        dip.releaseFreightPos();
        dip.releaseFreight();
        dip.getFreight();
        threadAuto.setElevatorState(ElevatorState.ZERO);
    }

    void goToMid() throws InterruptedException {
        threadAuto.setElevatorState(ElevatorThreadAuto.ElevatorState.MID);
        Thread.sleep(800);
        dip.releaseFreightPos();
        dip.releaseFreight();
        dip.getFreight();
        threadAuto.setElevatorState(ElevatorState.ZERO);
    }

    void goToMax() throws InterruptedException {
        threadAuto.setElevatorState(ElevatorThreadAuto.ElevatorState.MAX);
        Thread.sleep(800);
        dip.releaseFreightPos();
        dip.releaseFreight();
        dip.getFreight();
        threadAuto.setElevatorState(ElevatorState.ZERO);
    }

    void fixIntake() throws InterruptedException {
        intake.intakeBackward();
        Thread.sleep((long)(reverseIntakeFor * 1000));
        intake.stop();
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
}
