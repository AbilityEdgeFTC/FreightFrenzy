package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.opmodes.Vision.YCbCrPipeline;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorLibraryPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerLibraryPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "Left Red FULL", group = "Autonomous")
public class AutoLeftRed extends LinearOpMode {

    double startPoseLeftX = -35;
    double startPoseLeftY = -60;
    double startPoseLeftH = 90;
    public static double poseCarouselX = -62.2;
    public static double poseCarouselY = -59.5;
    public static double poseCarouselH = 95;
    public static double carouselHelp = 15;
    public static double poseParkHelpX = -56;
    public static double poseParkHelpY = -3;
    public static double poseParkHelpH = 180;
    public static double poseParkaX = 6;
    public static double poseParkaY = -3;
    public static double poseParkaH = 180;
    public static double poseParkbX = 7;
    public static double poseParkbY = -50;
    public static double poseParkbH = 180;
    public static double poseParkcX = 58;
    public static double poseParkcY = -50;
    public static double poseParkcH = 180;
    public static double runCarouselFor = 4;
    ElevatorFirstPID elevator;
    SpinnerFirstPID spinner;
    hand hand;
    intake intake;
    dip dip;
    carousel carousel;
    boolean canIntake = true;
    public static double powerSlowElevator = .6, powerElevator = 1, powerElevatorFast = 1;
    SampleMecanumDrive drive;
    TrajectorySequence main;

    enum levels
    {
        MIN,
        MID,
        MAX
    }

    levels placeFreightIn = levels.MAX;

    OpenCvWebcam webcam;
    YCbCrPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initPipeline();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        elevator = new ElevatorFirstPID(hardwareMap);
        spinner = new SpinnerFirstPID(hardwareMap);
        intake = new intake(hardwareMap);
        hand = new hand(hardwareMap);
        carousel = new carousel(hardwareMap);
        dip = new dip(hardwareMap);
        DriveConstants.setMaxVel(60);
        DriveConstants.setMaxVAcc(40);

        Pose2d startPoseLeft = new Pose2d(startPoseLeftX, startPoseLeftY, Math.toRadians(startPoseLeftH));
        Pose2d poseCarousel = new Pose2d(poseCarouselX, poseCarouselY, Math.toRadians(poseCarouselH));
        Pose2d poseParkingHelp = new Pose2d(poseParkHelpX,poseParkHelpY,Math.toRadians(poseParkHelpH));
        Pose2d poseParkinga = new Pose2d(poseParkaX, poseParkaY, Math.toRadians(poseParkaH));
        Pose2d poseParkingb = new Pose2d(poseParkbX, poseParkbY, Math.toRadians(poseParkbH));
        Pose2d poseParkingc = new Pose2d(poseParkcX, poseParkcY, Math.toRadians(poseParkcH));

        MarkerCallback carouselOnn = new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                carousel.spin(false,true);
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

        MarkerCallback elevetorDuckLevel3 = new MarkerCallback()
        {
            @Override
            public void onMarkerReached() {
                elevator.updateAuto();
                spinner.updateAuto();

                powerElevator = powerElevatorFast;
                elevator.setPower(powerElevatorFast);
                canIntake = false;

                dip.holdFreight();

                spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.DUCK_ANGLE_RED);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.DUCK_LEVEL);
                hand.level3Duck();

                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

        MarkerCallback elevetorClose =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                elevator.updateAuto();
                spinner.updateAuto();
                spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.DUCK_ANGLE_RED);
                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

        // half length and spinnner and close dip
        MarkerCallback elevetorVisionA = new MarkerCallback()
        {
            @Override
            public void onMarkerReached() {
                elevator.updateAuto();
                spinner.updateAuto();

                powerElevator = powerElevatorFast;
                elevator.setPower(powerElevatorFast);

                dip.holdFreight();

                switch (placeFreightIn)
                {
                    case MIN:
                    case MID:
                    case MAX:
                        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.LEFT_AUTO_ANGLE_RED);
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.MID);
                        break;
                }

                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

        // hand servo
        MarkerCallback elevetorVisionB = new MarkerCallback()
        {
            @Override
            public void onMarkerReached() {
                elevator.updateAuto();
                spinner.updateAuto();

                powerElevator = powerElevatorFast;
                elevator.setPower(powerElevatorFast);

                dip.holdFreight();

                switch (placeFreightIn)
                {
                    case MIN:
                        hand.level1Duck();
                        break;
                    case MID:
                        hand.level2Duck();
                        break;
                    case MAX:
                        hand.level3Duck();
                        break;
                }

                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

        // all length
        MarkerCallback elevetorVisionC = new MarkerCallback()
        {
            @Override
            public void onMarkerReached() {
                elevator.updateAuto();
                spinner.updateAuto();

                powerElevator = powerElevatorFast;
                elevator.setPower(powerElevatorFast);

                dip.holdFreight();

                switch (placeFreightIn)
                {
                    case MIN:
                    case MID:
                    case MAX:
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3);
                        break;
                }

                elevator.updateAuto();
                spinner.updateAuto();
            }
        };


        // also here, first dip servo relase, then half length elevator, then hand servo intake
        MarkerCallback elevetorCloseA =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                elevator.updateAuto();
                spinner.updateAuto();
                dip.releaseFreight();
                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

        MarkerCallback elevetorCloseB =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                elevator.updateAuto();
                spinner.updateAuto();
                powerElevator = powerSlowElevator;
                elevator.setPower(powerElevator);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.MID);
                hand.intake();
                elevator.updateAuto();
                spinner.updateAuto();
                elevator.updateAuto();
                spinner.updateAuto();
                hand.intake();
            }
        };

        MarkerCallback elevetorCloseC =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                elevator.updateAuto();
                spinner.updateAuto();
                powerElevator = powerSlowElevator;
                elevator.setPower(powerElevator);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

        drive.setPoseEstimate(startPoseLeft);

        /*
            SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
            SampleMecanumDrive.getAccelerationConstraint(20))
         */

        dip.getFreight();

        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("BARCODE LOCATION: ", pipeline.getLocation());
            switch (pipeline.getLocation())
            {
                case Left:
                    placeFreightIn = levels.MIN; // RED, blue = 3
                    break;
                case Center:
                    placeFreightIn = levels.MID; // RED, blue = 2
                    break;
                case Right:
                case Not_Found:
                    placeFreightIn = levels.MAX; // RED, blue = 1
                    break;
            }
            telemetry.update();
        }

        switch (placeFreightIn)
        {
            case MIN:
            case MID:
            case MAX:
                main = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(elevetorVisionA)
                    .waitSeconds(.8)
                    .addTemporalMarker(elevetorVisionB)
                    .waitSeconds(1.2)
                    .addTemporalMarker(elevetorVisionC)
                    .waitSeconds(.6)
                    .addTemporalMarker(elevetorCloseA)
                    .waitSeconds(.4)
                    .addTemporalMarker(elevetorCloseB)
                    .waitSeconds(1.5)
                    .addTemporalMarker(elevetorCloseC)
                    .waitSeconds(.5)
                    .forward(carouselHelp)
                    .lineToLinearHeading(poseCarousel)
                    .addTemporalMarker(carouselOnn)
                    .waitSeconds(runCarouselFor)
                    .addTemporalMarker(carouselOff)
                    .addTemporalMarker(intakeDuck)
                    .strafeLeft(5)
                    .back(6)
                    .strafeLeft(16)
                    .strafeRight(16)
                    .lineToLinearHeading(poseParkingHelp)
                    .addTemporalMarker(intakeStop)
                    .lineToLinearHeading(poseParkinga)
                    .addTemporalMarker(elevetorClose)
                    .addTemporalMarker(elevetorDuckLevel3)
                    .waitSeconds(.88)
                    .addTemporalMarker(elevetorCloseA)
                    .waitSeconds(.7)
                    .addTemporalMarker(elevetorCloseB)
                    .waitSeconds(.5)
                    .addTemporalMarker(elevetorCloseC)
                    .waitSeconds(.5)
                    .lineToLinearHeading(poseParkingb)
                    .waitSeconds(.6)
                    .lineToLinearHeading(poseParkingc,SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(80))
                    .build();
                break;
        }

        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.ZERO_RED);

        waitForStart();
        webcam.stopStreaming();
        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
        spinner.updateAuto();
        elevator.updateAuto();

        drive.followTrajectorySequence(main);
        spinner.updateAuto();
    }

    public void initPipeline()
    {
        //setting up webcam from config, and displaying it in the teleop controller.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //getting the pipeline and giving it telemetry. and setting the pipeline to the webcam
        pipeline = new YCbCrPipeline();
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
    }


}
