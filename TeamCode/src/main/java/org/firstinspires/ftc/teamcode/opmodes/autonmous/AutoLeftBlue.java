package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmodes.Vision.YCbCrPipeline;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "Left Blue FULL", group = "Autonomous")
public class AutoLeftBlue extends LinearOpMode {

    double startPoseRightX = 13;
    double startPoseRightY = 60;
    double startPoseRightH = 270;
    public static double poseEntranceX = 15;
    public static double poseEntranceY = 63.5;
    public static double poseEntranceH = 180;
    public static double poseCollectX = 47.5;
    public static double poseCollectY = 64;
    public static double poseCollectH = 180;
    public static double poseHelpX =9;
    public static double poseHelpY = 52;
    public static double poseHelpH = 180;
    ElevatorFirstPID elevator;
    SpinnerFirstPID spinner;
    hand hand;
    intake intake;
    dip dip;
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
        dip = new dip(hardwareMap);

        Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));
        Pose2d poseHelp = new Pose2d(poseHelpX, poseHelpY, Math.toRadians(poseHelpH));
        DriveConstants.setMaxVel(80);

        MarkerCallback elevetorVision = new MarkerCallback()
        {
            @Override
            public void onMarkerReached() {
                elevator.updateAuto();
                spinner.updateAuto();

                powerElevator = powerElevatorFast;
                elevator.setPower(powerElevatorFast);
                canIntake = false;

                dip.holdFreight();

                switch (placeFreightIn)
                {
                    case MIN:
                        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.LEFT);
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL1);
                        hand.level1();
                        break;
                    case MID:
                        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.LEFT);
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL2);
                        hand.level2();
                        break;
                    case MAX:
                        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.LEFT);
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3);
                        hand.level3();
                        break;
                }

                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

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

                spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.LEFT);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3);
                hand.level3();

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
                spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.LEFT);
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

        MarkerCallback intakeBackword =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                intake.intakeBackward();
                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

        drive.setPoseEstimate(startPoseRight);

        /*
            SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
            SampleMecanumDrive.getAccelerationConstraint(20))
         */

        main = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(poseHelp, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/1.5, DriveConstants.MAX_ANG_VEL/2, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(poseEntrance.getX()+0.5, poseEntrance.getY(), poseEntrance.getHeading()))
                .addTemporalMarker(elevetorVision)
                .waitSeconds(.8)
                .addDisplacementMarker(elevetorClose)
                .addTemporalMarker(intakeForward)
                .waitSeconds(0.3)
                .lineToSplineHeading(poseCollect)
                .waitSeconds(.7)
                .addTemporalMarker(intakeBackword)
                .lineToSplineHeading(poseEntrance)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.75)
                .addDisplacementMarker(elevetorClose)
                .addTemporalMarker(intakeForward)
                .waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(poseCollect.getX() + 3.5, poseCollect.getY(), poseCollect.getHeading()))
                .waitSeconds(.7)
                .addTemporalMarker(intakeBackword)
                .waitSeconds(.2)
                .lineToSplineHeading(poseEntrance)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.75)
                .addDisplacementMarker(elevetorClose)
                .addTemporalMarker(intakeForward)
                .waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(poseCollect.getX() + 7.5, poseCollect.getY(), poseCollect.getHeading()))
                .waitSeconds(.7)
                .addTemporalMarker(intakeBackword)
                .waitSeconds(.2)
                .lineToSplineHeading(poseEntrance)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.75)
                .addDisplacementMarker(elevetorClose)
                .addTemporalMarker(intakeForward)
                .waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(poseCollect.getX() + 10.5, poseCollect.getY(), poseCollect.getHeading()))
                .waitSeconds(.7)
                .addTemporalMarker(intakeBackword)
                .waitSeconds(.2)
                .lineToSplineHeading(poseEntrance)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.75)
                .addDisplacementMarker(elevetorClose)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(new Pose2d(poseCollect.getX() + 12, poseCollect.getY(), poseCollect.getHeading()))
                .build();

        dip.getFreight();

        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("BARCODE LOCATION: ", pipeline.getLocation());
            switch (pipeline.getLocation())
            {
                case Left:
                    placeFreightIn = levels.MIN;
                    break;
                case Center:
                    placeFreightIn = levels.MID;
                    break;
                case Right:
                    placeFreightIn = levels.MAX;
                    break;
                case Not_Found:
                    int random = (int)(Math.random() * 2) + 1;
                    switch (random)
                    {
                        case 1:
                            placeFreightIn = levels.MID; // RED, blue = 2
                            break;
                        case 2:
                            placeFreightIn = levels.MIN; // RED, blue = 2
                            break;
                    }
                    break;
            }
        }

        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.ZERO_BLUE);

        waitForStart();
        webcam.stopStreaming();
        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.LEFT);
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
        pipeline.setRedAlliance(false);

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
