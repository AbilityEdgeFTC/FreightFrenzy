package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.opmodes.Vision.HSVPipeline;
import org.firstinspires.ftc.teamcode.robot.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.robot.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.Cover;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "Left Red FULL", group = "Autonomous Red")
public class AutoLeftRed extends LinearOpMode {

    double startPoseLeftX = -35;
    double startPoseLeftY = -72 + 17.72;
    double startPoseLeftH = 90;

    public static double poseCarouselX = -63.4;
    public static double poseCarouselY = -51.6;
    public static double poseCarouselH = 5;
    public static double carouselHelp = 15;
    public static double runCarouselFor = 6.72;
    ElevatorFirstPID elevator;
    SpinnerFirstPID spinner;
    hand hand;
    intake intake;
    dip dip;
    Carousel carousel;
    Cover cover;
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
    HSVPipeline pipeline;

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
        carousel = new Carousel(hardwareMap);
        dip = new dip(hardwareMap);
        cover = new Cover(hardwareMap);
        DriveConstants.setMaxVel(60);
        DriveConstants.setMaxAccel(40);

        Pose2d startPoseLeft = new Pose2d(startPoseLeftX, startPoseLeftY, Math.toRadians(startPoseLeftH));
        Pose2d poseCarousel = new Pose2d(poseCarouselX, poseCarouselY, Math.toRadians(poseCarouselH));
        //Pose2d poseParking = new Pose2d(poseParkHelpX,poseParkHelpY,Math.toRadians(poseParkHelpH));

        MarkerCallback carouselOnn = new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                carousel.spinCarouselNoAccel(false, 0.18);
            }
        };
        MarkerCallback carouselOff = new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                carousel.stopCarouselNoAccel();
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
                cover.openCover();
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
                cover.closeCover();
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
                    .waitSeconds(.3)
                    .addTemporalMarker(elevetorVisionB)
                    .waitSeconds(.3)
                    .addTemporalMarker(elevetorVisionC)
                    .waitSeconds(.6)
                    .addTemporalMarker(elevetorCloseA)
                    .waitSeconds(.3)
                    .addTemporalMarker(elevetorCloseB)
                    .waitSeconds(.3)
                    .addTemporalMarker(elevetorCloseC)
                    .waitSeconds(.5)
                    .forward(carouselHelp)
                    .lineToLinearHeading(poseCarousel,SampleMecanumDrive.getVelocityConstraint(47, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(40))
                    .addTemporalMarker(carouselOnn)
                    .waitSeconds(runCarouselFor)
                    .addTemporalMarker(carouselOff)
                    .forward(8)
                    .turn(Math.toRadians(85))
                    .addTemporalMarker(intakeDuck)
                    .back(8)
                    .strafeRight(20)
                    .addTemporalMarker(intakeStop)
                    .back(3)
                    .addTemporalMarker(elevetorVisionA)
                    .waitSeconds(.3)
                    .addTemporalMarker(elevetorVisionB)
                    .waitSeconds(.3)
                    .addTemporalMarker(elevetorVisionC)
                    .waitSeconds(.6)
                    .addTemporalMarker(elevetorCloseA)
                    .waitSeconds(.3)
                    .addTemporalMarker(elevetorCloseB)
                    .waitSeconds(.3)
                    .addTemporalMarker(elevetorCloseC)
                    .waitSeconds(.5)
                    .forward(25)
                    .turn(Math.toRadians(-90))
                    .back(17)
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
        ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("ElevatorEncoderValue.txt"), "" + elevator.getPosition());
    }

    public void initPipeline()
    {
        //setting up webcam from config, and displaying it in the teleop controller.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //getting the pipeline and giving it telemetry. and setting the pipeline to the webcam
        pipeline = new HSVPipeline();
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
