package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

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
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
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
@Autonomous(name = "Left Red TEST", group = "Autonomous")
public class AutoLeftRedTEST extends LinearOpMode {

    double startPoseLeftX = -35;
    double startPoseLeftY = -60;
    double startPoseLeftH = 90;
    public static double poseCarouselX = -62.2;
    public static double poseCarouselY = -59.2;
    public static double poseCarouselH = 95;
    public static double carouselHelp = 15;
    public static double poseParkHelpX = -27.5;
    public static double poseParkHelpY = -9;
    public static double poseParkHelpH = 180;
    public static double poseParkaX = 13;
    public static double poseParkaY = -9;
    public static double poseParkaH = 180;
    public static double poseParkbX = 13;
    public static double poseParkbY = -50;
    public static double poseParkbH = 180;
    public static double poseParkcX = 65;
    public static double poseParkcY = -50;
    public static double poseParkcH = 180;
    public static double runCarouselFor = 4;
    ElevatorFirstPID elevator;
    SpinnerFirstPID spinner;
    carousel carousel;
    hand hand;
    intake intake;
    dip dip;
    boolean canIntake = true;
    public static double powerSlowElevator = .6, powerElevator = 1, powerElevatorFast = 1;
    SampleMecanumDrive drive;
    TrajectorySequence main;

    public static int elevatorLevel = 4;

    //OpenCvWebcam webcam;
    //YCbCrPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //        ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("RRheadingValue.txt"), "" + Math.toRadians(startPoseRightH));
        ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("ElevatorValue.txt"), "" + 0);
//        ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("SpinnerValue.txt"), "" + 0);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        elevator = new ElevatorFirstPID(hardwareMap);
        spinner = new SpinnerFirstPID(hardwareMap);
        //spinner = new ElevatorSpinnerLibraryPID(hardwareMap);
        intake = new intake(hardwareMap);
        hand = new hand(hardwareMap);
        dip = new dip(hardwareMap);
        carousel = new carousel(hardwareMap);

        Pose2d startPoseLeft = new Pose2d(startPoseLeftX, startPoseLeftY, Math.toRadians(startPoseLeftH));
        Pose2d poseCarousel = new Pose2d(poseCarouselX, poseCarouselY, Math.toRadians(poseCarouselH));
        Pose2d poseParkingHelp = new Pose2d(poseParkHelpX,poseParkHelpY,Math.toRadians(poseParkHelpH));
        Pose2d poseParkinga = new Pose2d(poseParkaX, poseParkaY, Math.toRadians(poseParkaH));
        Pose2d poseParkingb = new Pose2d(poseParkbX, poseParkbY, Math.toRadians(poseParkbH));
        Pose2d poseParkingc = new Pose2d(poseParkcX, poseParkcY, Math.toRadians(poseParkcH));

        MarkerCallback elevetorOpen = new MarkerCallback()
        {
            @Override
            public void onMarkerReached() {
                elevator.updateAuto();
                spinner.updateAuto();

                powerElevator = powerElevatorFast;
                elevator.setPower(powerElevatorFast);
                canIntake = false;

                dip.holdFreight();

                switch (elevatorLevel)
                {
                    case 1:
                        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL1);
                        hand.level1();
                        break;
                    case 2:
                        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL2);
                        hand.level2();
                        break;
                    case 3:
                        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3);
                        hand.level3();
                        break;
                    case 4:
                        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.LEFT_AUTO_ANGLE);
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.AUTO_LEFT_LEVEL);
                        hand.level3();
                        break;
                    case 5:
                        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.DUCK_ANGLE);
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.DUCK_RED_LEVEL);
                        hand.level3();
                        break;
                }

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
                powerElevator = powerSlowElevator;
                dip.releaseFreight();
                elevator.updateAuto();
                spinner.updateAuto();
                canIntake = true;
                elevator.setPower(powerElevator);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
                hand.intake();
                spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

        MarkerCallback intakeForward =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                dip.getFreight();
                intake.intakeForward();
                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

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
        drive.setPoseEstimate(startPoseLeft);

        /*
            SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
            SampleMecanumDrive.getAccelerationConstraint(20))
         */

        //initPipeline();

        main = drive.trajectorySequenceBuilder(drive.getPoseEstimate())

                .addTemporalMarker(elevatorLevel = 4,elevetorOpen)
                .waitSeconds(1.5)
                .addTemporalMarker(elevetorClose)
                .waitSeconds(2)
                .forward(carouselHelp)
                .lineToLinearHeading(poseCarousel)
                .addTemporalMarker(carouselOnn)
                .waitSeconds(runCarouselFor)
                .addTemporalMarker(carouselOff)
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
                .lineToSplineHeading(poseParkingHelp)
                .splineToLinearHeading(poseParkinga,poseParkaH)
                .addTemporalMarker(elevatorLevel = 5,elevetorOpen)
                .waitSeconds(1)
                .addTemporalMarker(elevetorClose)
                .lineToLinearHeading(poseParkingb)
                .waitSeconds(.6)
                .lineToLinearHeading(poseParkingc,SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(80))
                .build();

        dip.getFreight();

        /*while (!opModeIsActive())
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
                    placeFreightIn = levels.MAX; // RED, blue = 1
                    break;
            }
        }*/
        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.ZERO_RED);

        waitForStart();
        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
        spinner.updateAuto();
        elevator.updateAuto();

        drive.followTrajectorySequence(main);
        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
        spinner.updateAuto();

//        ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("RRheadingValue.txt"), "" + drive.getExternalHeading());
        ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("ElevatorValue.txt"), "" + elevator.getPosition());
//        ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("SpinnerValue.txt"), "" + spinner.getPosition());
    }

    /*public void initPipeline()
    {
        //setting up webcam from config, and displaying it in the teleop controller.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //getting the pipeline and giving it telemetry. and setting the pipeline to the webcam
        pipeline = new YCbCrPipeline();
        pipeline.telemetry = telemetry;
        pipeline.DEBUG = false;
        pipeline.setRedAlliance(true);


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
