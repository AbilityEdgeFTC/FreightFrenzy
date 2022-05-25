package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import static org.firstinspires.ftc.teamcode.opmodes.autonmous.AutoLeftBlue.poseEntranceX;
import static org.firstinspires.ftc.teamcode.opmodes.autonmous.AutoLeftBlue.poseEntranceY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmodes.Vision.HSVPipeline;
import org.firstinspires.ftc.teamcode.robot.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.Cover;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "AutoTesting", group = "Autonomous  Red")
@Disabled
public class AutoTesting extends LinearOpMode {

    double startPoseRightX = 13;
    double startPoseRightY = -62;
    double startPoseRightH = 90;
    double poseEntranceX = 13;
    double poseEntranceY = -64;
    double poseEntranceH = 180;
    double poseCollectX = 50;
    double poseCollectY = -64;
    double poseCollectH = 180;
    double poseHelpX = 7;
    double poseHelpY = -50;
    double poseHelpH = 180;

    //Cordinates for each course
    double cylceX2 = 60;
    double cycleY2 = -59;
    double cycleH2 = 45;

    double cylceX3 = 60;
    double cycleY3 = -63;
    double cycleH3 = -45;


    ElevatorFirstPID elevator;
    SpinnerFirstPID spinner;
    hand hand;
    dip dip;
    Cover cover;

    public static double powerSlowElevator = .6, powerElevator = 1, powerElevatorFast = 1;
    SampleMecanumDrive drive;
    TrajectorySequence main;


    enum levels {
        MIN,
        MID,
        MAX
    }

    levels placeFreightIn = levels.MAX;

    OpenCvWebcam webcam;
    HSVPipeline pipeline;

    MarkerCallback openCover = new MarkerCallback() {
        @Override
        public void onMarkerReached() {
            elevator.updateAuto();
            spinner.updateAuto();

            powerElevator = powerElevatorFast;
            elevator.setPower(powerElevatorFast);

            dip.holdFreight();
            cover.openCover();

        }

    };

    MarkerCallback elevatorOpen = new MarkerCallback() {
        @Override
        public void onMarkerReached() {
            elevator.updateAuto();
            spinner.updateAuto();


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
            elevator.setPower(powerElevator);
            elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
            elevator.updateAuto();
            spinner.updateAuto();
            cover.closeCover();
        }
    };
    @Override
    public void runOpMode() throws InterruptedException {
        initPipeline();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        elevator = new ElevatorFirstPID(hardwareMap);
        spinner = new SpinnerFirstPID(hardwareMap);
        hand = new hand(hardwareMap);
        dip = new dip(hardwareMap);
        cover = new Cover(hardwareMap);


        Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        Pose2d poseHelp = new Pose2d(poseHelpX, poseHelpY, Math.toRadians(poseHelpH));
        Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));
        Pose2d poseCollectCycle2 = new Pose2d(cylceX2 , cycleY2, Math.toRadians(cycleH2));
        Pose2d poseCollectCycle3 = new Pose2d(cylceX3 , cycleY3, Math.toRadians(cycleH3));



        DriveConstants.setMaxVel(70);

       // main = drive.trajectorySequenceBuilder(startPoseRight)
              //  .lineToLinearHeading(poseHelp)
                //.lineToSplineHeading(poseEntrance)
               // .addTemporalMarker(openCover)
              //  .waitSeconds(1)
               // .addTemporalMarker(elevatorOpen)
              // .waitSeconds(.8)
              //  .addTemporalMarker(elevetorClose)
              //  .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
               // .lineToSplineHeading(poseEntrance)
              //  .addTemporalMarker(openCover)
               // .waitSeconds(1)
              //  .addTemporalMarker(elevatorOpen)
              //  .waitSeconds(.8)
              //  .addTemporalMarker(elevetorClose)
               // .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
               // .lineToConstantHeading(new Vector2d(poseCollectCycle2.getX(), poseCollectCycle2.getY()))
              //  .waitSeconds(.8)
              //  .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                //.lineToSplineHeading(poseEntrance)
              //  .addTemporalMarker(openCover)
            //    .waitSeconds(1)
              //  .addTemporalMarker(elevatorOpen)
               // .waitSeconds(.8)
              //  .addTemporalMarker(elevetorClose)
               // .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
            //    .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY() + 5, poseCollect.getHeading()))
               // .waitSeconds(.8)
               // .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
              //  .lineToSplineHeading(poseEntrance)
             //   .addTemporalMarker(openCover)
            //    .waitSeconds(1)
             //   .addTemporalMarker(elevatorOpen)
              //  .waitSeconds(.8)
             //   .addTemporalMarker(elevetorClose)
              //  .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
              //  .lineToConstantHeading(new Vector2d(poseCollectCycle2.getX() + 2, poseCollectCycle2.getY() - 15))
               // .waitSeconds(.8)
               // .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
             //   .lineToSplineHeading(poseEntrance)
            //    .addTemporalMarker(openCover)
            //    .waitSeconds(1)
              //  .addTemporalMarker(elevatorOpen)
             //   .waitSeconds(.8)
                //.addTemporalMarker(elevetorClose)
             //   .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
              //  .lineToSplineHeading(new Pose2d(poseCollect.getX() + 4, poseCollect.getY(), poseCollect.getHeading()))
            //    .waitSeconds(.8)
                //.lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
               // .lineToSplineHeading(poseEntrance)
                //.addTemporalMarker(openCover)
                //.waitSeconds(1)
              //  .addTemporalMarker(elevatorOpen)
               // .waitSeconds(.8)
               // .addTemporalMarker(elevetorClose)
              //  .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
           //     .build();




        // half length and spinnner and close dip
        MarkerCallback elevetorVisionA = new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                elevator.updateAuto();
                spinner.updateAuto();

                powerElevator = powerElevatorFast;
                elevator.setPower(powerElevatorFast);

                dip.holdFreight();

                switch (placeFreightIn) {
                    case MIN:
                        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.MID);
                        hand.level1();
                        break;
                    case MID:
                        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.MID);
                        hand.level2();
                        break;
                    case MAX:
                        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.MID);
                        hand.level3();
                        break;
                }

                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

        // hand servo
        MarkerCallback elevetorVisionB = new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                elevator.updateAuto();
                spinner.updateAuto();

                powerElevator = powerElevatorFast;
                elevator.setPower(powerElevatorFast);

                dip.holdFreight();

                switch (placeFreightIn) {
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
        MarkerCallback elevetorVisionC = new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                elevator.updateAuto();
                spinner.updateAuto();

                powerElevator = powerElevatorFast;
                elevator.setPower(powerElevatorFast);

                dip.holdFreight();

                switch (placeFreightIn) {
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
        MarkerCallback elevetorCloseA = new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                elevator.updateAuto();
                spinner.updateAuto();
                dip.releaseFreight();
                spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

        MarkerCallback elevetorCloseB = new MarkerCallback() {
            @Override
            public void onMarkerReached() {
                elevator.updateAuto();
                spinner.updateAuto();
                powerElevator = powerSlowElevator;
                elevator.setPower(powerElevator);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.MID);
                elevator.updateAuto();
                spinner.updateAuto();
                spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                elevator.updateAuto();
                spinner.updateAuto();
                hand.intake();
            }
        };

        MarkerCallback elevetorCloseC = new MarkerCallback() {
            @Override
            public void onMarkerReached() {
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

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("BARCODE LOCATION: ", pipeline.getLocation());
            switch (pipeline.getLocation()) {
                case Left:
                    placeFreightIn = levels.MIN; // RED, blue = 3
                    break;
                case Center:
                    placeFreightIn = levels.MID; // RED, blue = 2
                    break;
                case Right:
                    placeFreightIn = levels.MAX; // RED, blue = 1
                    break;
                case Not_Found:
                    int random = (int) (Math.random() * 2) + 1;
                    switch (random) {
                        case 1:
                            placeFreightIn = levels.MID; // RED, blue = 2
                            break;
                        case 2:
                            placeFreightIn = levels.MIN; // RED, blue = 2
                            break;
                    }
                    break;
            }
            telemetry.update();
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

    public void initPipeline() {
        //setting up webcam from config, and displaying it in the teleop controller.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        //getting the pipeline and giving it telemetry. and setting the pipeline to the webcam
        pipeline = new HSVPipeline();
        pipeline.telemetry = telemetry;
        pipeline.DEBUG = false;


        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 0);
            }

            @Override
            public void onError(int errorCode) {

            }
        });




    }








}
