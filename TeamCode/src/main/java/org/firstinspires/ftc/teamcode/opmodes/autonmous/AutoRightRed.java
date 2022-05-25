package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.opmodes.Vision.HSVPipeline;
import org.firstinspires.ftc.teamcode.robot.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
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
@Autonomous(name = "Right Red", group = "Autonomous Red")
public class AutoRightRed extends LinearOpMode {

    double startPoseRightX = 13;
    double startPoseRightY = -72 + 17.72;
    double startPoseRightH = 90;

    public static double poseEntranceX = 13;
    public static double poseEntranceY = -58;
    public static double poseEntranceH = 180;
    public static double poseCollectX = 50;
    public static double poseCollectY = -58;
    public static double poseCollectH = 180;
    public static double poseHelpX = 7;
    public static double poseHelpY = -50;
    public static double poseHelpH = 180;

    //Cordinates for each course
    public static double cylceX2 = 60;
    public static double cycleY2 = -58;
    public static double cycleH2 = 180;

    public static double cylceX3 = 70;
    public static double cycleY3 = -55;
    public static double cycleH3 = 180;

    ElevatorFirstPID elevator;
    SpinnerFirstPID spinner;
    hand hand;
    intake intake;
    dip dip;
    Cover cover;
    boolean canIntake = true;
    public static double powerSlowElevator = .6, powerElevator = 1, powerElevatorFast = 1;
    SampleMecanumDrive drive;
    TrajectorySequence main;
    boolean useVision = true;

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
        if(useVision)
        {
            initPipeline();
        }
        else
        {
            placeFreightIn = levels.MAX;
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        elevator = new ElevatorFirstPID(hardwareMap);
        spinner = new SpinnerFirstPID(hardwareMap);
        cover = new Cover(hardwareMap);
        intake = new intake(hardwareMap);
        hand = new hand(hardwareMap);
        dip = new dip(hardwareMap);


        Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        Pose2d poseHelp = new Pose2d(poseHelpX, poseHelpY, Math.toRadians(poseHelpH));
        Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));
        Pose2d poseCollectCycle2 = new Pose2d(cylceX2 , cycleY2, Math.toRadians(cycleH2));
        Pose2d poseCollectCycle3 = new Pose2d(cylceX3 , cycleY3, Math.toRadians(cycleH3));

        MarkerCallback elevetorOpen = new MarkerCallback()
        {
            @Override
            public void onMarkerReached() {
                cover.openCover();

                elevator.updateAuto();
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3);
                spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                spinner.updateAuto();

                powerElevator = powerElevatorFast;
                elevator.setPower(powerElevatorFast);
                canIntake = false;

                dip.holdFreight();

                hand.level3();

                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

        MarkerCallback elevetorClose =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                dip.releaseFreight();

                elevator.updateAuto();
                spinner.updateAuto();
                powerElevator = powerSlowElevator;
                dip.releaseFreight();
                elevator.updateAuto();
                spinner.updateAuto();
                canIntake = true;
                elevator.setPower(powerElevator);
                hand.intake();
                spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
                elevator.updateAuto();
                spinner.updateAuto();

                cover.closeCover();
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
                        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.MID);
                        break;
                }

                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

        MarkerCallback intakeStop =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                intake.stop();
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
                        hand.level1();
                        break;
                    case MID:
                        hand.level2();
                        break;
                    case MAX:
                        hand.level3();
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
                spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                powerElevator = powerSlowElevator;
                elevator.setPower(powerElevator);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.MID);
                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

        MarkerCallback elevetorCloseB =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                dip.getFreight();
                powerElevator = powerSlowElevator;
                elevator.setPower(powerElevator);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.MID);
                elevator.updateAuto();
                spinner.updateAuto();
                elevator.updateAuto();
                spinner.updateAuto();
                hand.intake();
                spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

        MarkerCallback elevetorCloseC =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                dip.getFreight();
                elevator.updateAuto();
                spinner.updateAuto();
                powerElevator = powerSlowElevator;
                elevator.setPower(powerElevator);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
                spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

        drive.setPoseEstimate(startPoseRight);

        /*
            SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
            SampleMecanumDrive.getAccelerationConstraint(20))
         */

        dip.getFreight();
        cover.closeCover();

        while (!opModeIsActive() && !isStopRequested() && useVision)
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
//                main = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
//                        .lineToLinearHeading(poseHelp, SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL/1.5, DriveConstants.MAX_ANG_VEL/2, DriveConstants.TRACK_WIDTH),
//                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                        .lineToSplineHeading(new Pose2d(poseEntrance.getX()+2.5, poseEntrance.getY(), poseEntrance.getHeading()))
//                        .addTemporalMarker(elevetorVisionA)
//                        .waitSeconds(.2)
//                        .addTemporalMarker(elevetorVisionB)
//                        .waitSeconds(1)
//                        .addTemporalMarker(elevetorVisionC)
//                        .waitSeconds(.6)
//                        .addTemporalMarker(elevetorCloseA)
//                        .waitSeconds(.8)
//                        .addTemporalMarker(elevetorCloseB)
//                        .waitSeconds(2)
//                        .addTemporalMarker(elevetorCloseC)
//                        .addTemporalMarker(intakeStop)
//                        .waitSeconds(.2)
//                        .lineToSplineHeading(poseCollect, SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                SampleMecanumDrive.getAccelerationConstraint(30))
//                        .waitSeconds(2)
//                        .addTemporalMarker(intakeBackword)
//                        .waitSeconds(.2)
//                        .lineToSplineHeading(new Pose2d(poseEntrance.getX()+4.5, poseEntrance.getY(), poseEntrance.getHeading()))
//                        .addTemporalMarker(elevetorOpen)
//                        .waitSeconds(.75)
//                        .addDisplacementMarker(elevetorClose)
//                        .addTemporalMarker(intakeForward)
//                        .lineToSplineHeading(poseCollect, SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                SampleMecanumDrive.getAccelerationConstraint(30))
//                        .waitSeconds(2)
//                        .addTemporalMarker(intakeBackword)
//                        .waitSeconds(.2)
//                        .lineToSplineHeading(new Pose2d(poseEntrance.getX()+4.5, poseEntrance.getY(), poseEntrance.getHeading()))
//                        .addTemporalMarker(elevetorOpen)
//                        .waitSeconds(.75)
//                        .addDisplacementMarker(elevetorClose)
//                        .addTemporalMarker(intakeStop)
//                        .lineToSplineHeading(poseCollect, SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                SampleMecanumDrive.getAccelerationConstraint(30))
//                        .build();
//                break;
            case MAX:
                main = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(poseHelp)
                        .lineToSplineHeading(poseEntrance)
                        .addTemporalMarker(elevetorOpen)
                        .waitSeconds(.8)
                        .addTemporalMarker(elevetorClose)
                        .addTemporalMarker(intakeForward)
                        .lineToSplineHeading(new Pose2d(poseCollect.getX()+2, poseCollect.getY(), poseCollect.getHeading()),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(50))
                        .waitSeconds(.8)
                        .addTemporalMarker(intakeBackword)
                        .lineToSplineHeading(poseEntrance)
                        .addTemporalMarker(intakeStop)
                        .addTemporalMarker(elevetorOpen)
                        .waitSeconds(.8)
                        .addTemporalMarker(elevetorClose)
                        .addTemporalMarker(intakeForward)
                        .lineToSplineHeading(poseCollect)
                        .lineToSplineHeading(poseCollectCycle2,SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(50))
                        .waitSeconds(.8)
                        .lineToSplineHeading(new Pose2d(poseCollect.getX()+5, poseCollect.getY(), poseCollect.getHeading()),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(50))
                        .addTemporalMarker(intakeBackword)
                        .lineToSplineHeading(poseEntrance)
                        .addTemporalMarker(intakeStop)
                        .addTemporalMarker(elevetorOpen)
                        .waitSeconds(.8)
                        .addTemporalMarker(elevetorClose)
                        .addTemporalMarker(intakeForward)
                        .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                        .lineToSplineHeading(new Pose2d(poseCollect.getX()+8, poseCollect.getY()+8, poseCollect.getHeading()),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(50))
                        .waitSeconds(.8)
                        .addTemporalMarker(intakeBackword)
                        .lineToSplineHeading(new Pose2d(poseCollect.getX()+6, poseCollect.getY(), poseCollect.getHeading()),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(50))
                        .forward(4)
                        .lineToSplineHeading(poseEntrance)
                        .addTemporalMarker(intakeStop)
                        .addTemporalMarker(elevetorOpen)
                        .waitSeconds(.8)
                        .addTemporalMarker(elevetorClose)
                        .addTemporalMarker(intakeForward)
                        .lineToSplineHeading(poseCollect)
                        .lineToSplineHeading(poseCollectCycle3,SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(50))
                        .waitSeconds(.8)
                        .addTemporalMarker(intakeBackword)
                        .lineToSplineHeading(new Pose2d(poseCollect.getX()+15, poseCollect.getY(), poseCollect.getHeading()),SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(40))
                        .lineToSplineHeading(new Pose2d(poseEntrance.getX()-3, poseCollect.getY(), poseCollect.getHeading()))
                        .addTemporalMarker(intakeStop)
                        .addTemporalMarker(elevetorOpen)
                        .waitSeconds(.8)
                        .addTemporalMarker(elevetorClose)
                        .addTemporalMarker(intakeStop)
//                                .lineToSplineHeading(new Pose2d(poseEntrance.getX()+30, poseEntrance.getY(), poseEntrance.getHeading()))
//                                .lineToSplineHeading(new Pose2d(poseCollect.getX()+10, poseCollect.getY()-1.5, Math.toRadians(160)))
//                                .waitSeconds(.8)
//                                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
//                                .lineToSplineHeading(poseEntrance)
//                                .waitSeconds(.8)
                        .lineToSplineHeading(new Pose2d(poseCollect.getX()+10, poseCollect.getY(), poseCollect.getHeading()))
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
