package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.opmodes.Vision.HSVPipeline;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerLibraryPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;

@Config
@Autonomous(name = "Left Red FULL", group = "red")
public class AutoLeftRed extends LinearOpMode {

    public static double startPoseLeftX = -35;
    public static double startPoseLeftY = -60;
    public static double startPoseLeftH = 90;
    public static double poseCarouselX = -60.2;
    public static double poseCarouselY = -58.2;
    public static double poseCarouselH = 95;
    public static double carouselHelp = 15;
    public static double poseParkHelpX = -27.5;
    public static double poseParkHelpY = -9;
    public static double poseParkHelpH = 180;
    public static double poseParkaX = 9;
    public static double poseParkaY = -9;
    public static double poseParkaH = 180;
    public static double poseParkbX = 9;
    public static double poseParkbY = -50;
    public static double poseParkbH = 180;
    public static double poseParkcX = 65;
    public static double poseParkcY = -50;
    public static double poseParkcH = 180;
    public static double runCarouselFor = 4;

    carousel carousel;
    intake intake;
    dip dip;
    //OpenCvWebcam webcam;
    HSVPipeline pipeline;
    SampleMecanumDrive drive;

    //public static boolean withVision = true;

    /*enum levels
    {
        MIN,
        MID,
        MAX
    }

    levels placeFreightIn = levels.MAX;*/

    TrajectorySequence carouselGo,parking,duck;

    @Override
    public void runOpMode() throws InterruptedException {
        pipeline = new HSVPipeline();
        pipeline.telemetry = telemetry;
        pipeline.DEBUG = false;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);

        MarkerCallback carouselOnn = new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                 carousel.spin(true,true);
            }
        };
        MarkerCallback carouselOff = new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                carousel.stop();
            }
        };
        MarkerCallback intakeDuck =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                intake.intakeForward();
            }
        };
        MarkerCallback intakeStop =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                intake.stop();
            }
        };

        Pose2d startPoseLeft = new Pose2d(startPoseLeftX, startPoseLeftY, Math.toRadians(startPoseLeftH));
        Pose2d poseCarousel = new Pose2d(poseCarouselX, poseCarouselY, Math.toRadians(poseCarouselH));
        Pose2d poseParkingHelp = new Pose2d(poseParkHelpX,poseParkHelpY,Math.toRadians(poseParkHelpH));
        Pose2d poseParkinga = new Pose2d(poseParkaX, poseParkaY, Math.toRadians(poseParkaH));
        Pose2d poseParkingb = new Pose2d(poseParkbX, poseParkbY, Math.toRadians(poseParkbH));
        Pose2d poseParkingc = new Pose2d(poseParkcX, poseParkcY, Math.toRadians(poseParkcH));

        carousel = new carousel(hardwareMap);
        intake = new intake(hardwareMap);
        dip = new dip(hardwareMap);
        //threadAuto = new ElevatorThreadAuto(hardwareMap);

        //initPipeline();
        //webcam.setPipeline(pipeline);

        drive.setPoseEstimate(startPoseLeft);


        carouselGo = drive.trajectorySequenceBuilder(startPoseLeft)
                .forward(carouselHelp)
                .lineToLinearHeading(poseCarousel)
                .addTemporalMarker(carouselOnn)
                .waitSeconds(runCarouselFor)
                .addTemporalMarker(carouselOff)
                .build();

        duck = drive.trajectorySequenceBuilder(carouselGo.end())
                .addTemporalMarker(intakeDuck)
                .strafeRight(2)
                .back(6)
                .forward(8)
                .strafeRight(2)
                .back(7)
                .forward(8)
                .strafeRight(2)
                .back(8)
                .forward(8)
                .addTemporalMarker(intakeStop)
                .build();

        parking = drive.trajectorySequenceBuilder(duck.end())

                .lineToSplineHeading(poseParkingHelp)
                .splineToLinearHeading(poseParkinga,poseParkaH)
                .lineToLinearHeading(poseParkingb)
                .waitSeconds(.6)
                .lineToLinearHeading(poseParkingc,SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(80))

                .build();

        //threadAuto.start();
        //dip.getFreight();

        //spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.ZERO_RED);

        waitForStart();

        drive.followTrajectorySequence(carouselGo);
        drive.followTrajectorySequence(duck);
        drive.followTrajectorySequence(parking);
    }

    /*void runCarousel() throws InterruptedException
    {
        carousel.spin(true);
        sleep((long)(runCarouselFor * 1000));
        carousel.stop();
    }*/

    /*void goToMin() throws InterruptedException
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
    }*/

    /*void initPipeline()
    {
        //setting up webcam from config, and displaying it in the teleop controller.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //getting the pipeline and giving it telemetry. and setting the pipeline to the webcam
        HSVPipeline pipeline = new HSVPipeline();
        pipeline.telemetry = telemetry;
        pipeline.DEBUG = false;

        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {

                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam,0);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }*/
}
