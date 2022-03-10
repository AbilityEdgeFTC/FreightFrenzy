package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerLibraryPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
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
    public static double poseEntranceY = -62;
    public static double poseEntranceH = 180;
    public static double poseCollectX = 63;
    public static double poseCollectY = -62;
    public static double poseCollectH = 180;
    ElevatorFirstPID elevator;
    ElevatorSpinnerLibraryPID spinner;
    ElapsedTime resetElevator;
    hand hand;
    intake intake;
    dip dip;
    boolean canIntake = true, openElevator = false, closeElevator = false;
    public static double powerIntake = 1, powerSlowElevator = .6, powerElevator = 1;
    //public static double reverseIntakeFor = .8;
    //OpenCvWebcam webcam;
    //YCbCrPipeline pipeline;
    SampleMecanumDrive drive;
    TrajectorySequence main;

    //public static boolean withVision = true;

    enum levels
    {
        MIN,
        MID,
        MAX
    }

    levels placeFreightIn = levels.MAX;

    enum ElevatorMovement
    {
        SPIN,
        LEVEL1,
        LEVEL2,
        LEVEL3,
        DIP
    }

    public static int elevatorLevel = 3;

    public static ElevatorMovement elevatorMovement = ElevatorMovement.SPIN;

    @Override
    public void runOpMode() throws InterruptedException {
        //pipeline = new YCbCrPipeline();
        //pipeline.telemetry = telemetry;
        //pipeline.DEBUG = false;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        elevator = new ElevatorFirstPID(hardwareMap);
        spinner = new ElevatorSpinnerLibraryPID(hardwareMap);
        intake = new intake(hardwareMap);
        hand = new hand(hardwareMap);
        dip = new dip(hardwareMap);

        Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));

        intake = new intake(hardwareMap);
        //initPipeline();
        //webcam.setPipeline(pipeline);

        drive.setPoseEstimate(startPoseRight);

        main = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    openElevator = true;
                    switch (placeFreightIn)
                    {
                        case MIN:
                            elevatorLevel = 1;
                            break;
                        case MID:
                            elevatorLevel = 2;
                            break;
                        case MAX:
                            elevatorLevel = 3;
                            break;
                    }
                })
                .waitSeconds(2)
                .forward(5)
                .lineToSplineHeading(poseEntrance,
                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    if(canIntake)
                    {
                        intake.intakeForward();
                    }
                })
                .lineToSplineHeading(poseEntrance)
                .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> {
                    if(canIntake)
                    {
                        intake.intakeBackward();
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    openElevator = true;
                    elevatorLevel = 3;
                })
                .waitSeconds(2)
                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    if(canIntake)
                    {
                        intake.intakeForward();
                    }
                })
                .lineToSplineHeading(poseEntrance)
                .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> {
                    if(canIntake)
                    {
                        intake.intakeBackward();
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    openElevator = true;
                    elevatorLevel = 3;
                })
                .waitSeconds(2)
                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    if(canIntake)
                    {
                        intake.intakeForward();
                    }
                })
                .lineToSplineHeading(poseEntrance)
                .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> {
                    if(canIntake)
                    {
                        intake.intakeBackward();
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    openElevator = true;
                    elevatorLevel = 3;
                })
                .waitSeconds(2)
                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    if(canIntake)
                    {
                        intake.intakeForward();
                    }
                })
                .lineToSplineHeading(poseEntrance)
                .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> {
                    if(canIntake)
                    {
                        intake.intakeBackward();
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    openElevator = true;
                    elevatorLevel = 3;
                })
                .waitSeconds(2)
                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    if(canIntake)
                    {
                        intake.intakeForward();
                    }
                })
                .lineToSplineHeading(poseEntrance)
                .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> {
                    if(canIntake)
                    {
                        intake.intakeBackward();
                    }
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    openElevator = true;
                    elevatorLevel = 3;
                })
                .waitSeconds(2)
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

        resetElevator = new ElapsedTime();
        drive.followTrajectorySequenceAsync(main);

        waitForStart();

        /*if(withVision){
            webcam.stopStreaming();
        }*/

        while (opModeIsActive())
        {
            switchElevator();
            drive.update();

        }
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

    void switchElevator()
    {
        switch (elevatorMovement) {
            case SPIN:
                resetElevator();
                elevator.updateAuto();
                spinner.updateAuto();

                if (openElevator)
                {
                    powerElevator = powerSlowElevator;
                    elevator.setPower(powerElevator);
                    canIntake = false;

                    if(!withoutPID())
                    {
                        dip.holdFreight();
                    }

                    openElevator = false;

                    switch (elevatorLevel)
                    {
                        case 1:
                            elevatorMovement = ElevatorMovement.LEVEL1;
                            break;
                        case 2:
                            elevatorMovement = ElevatorMovement.LEVEL2;
                            break;
                        case 3:
                            elevatorMovement = ElevatorMovement.LEVEL3;
                            break;
                        default:
                            elevator.setUsePID(false);
                            break;
                    }
                }
                break;
            case LEVEL1:
                spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL1);
                if(!withoutPID())
                {
                    hand.level1();
                }
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case LEVEL2:
                spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL2);
                if(!withoutPID())
                {
                    hand.level2();
                }
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case LEVEL3:
                spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3);
                if(!withoutPID())
                {
                    hand.level3();
                }
                elevatorMovement = ElevatorMovement.DIP;
                break;
            case DIP:
                elevator.updateAuto();
                spinner.updateAuto();

                resetElevator.reset();

                if(resetElevator.seconds() > 0.5)
                {
                    closeElevator = true;
                }

                if(closeElevator)
                {
                    if(!withoutPID())
                    {
                        dip.releaseFreight();
                    }

                    resetElevator.reset();

                    canIntake = true;
                    closeElevator = false;
                    elevatorMovement = ElevatorMovement.SPIN;
                }
                break;
            default:
                elevatorMovement = ElevatorMovement.SPIN;
                break;
        }
    }

    boolean withoutPID()
    {
        if(elevator.getUsePID() == true || spinner.getUsePID() == true && elevator.getElevatorLevel() != ElevatorFirstPID.ElevatorLevel.ZERO)
        {
            switch (hand.getHandPos())
            {
                case INTAKE:
                    hand.intake();
                    break;
                case ONE_HUB:
                    hand.level1();
                    break;
                case TWO_HUB:
                    hand.level2();
                    break;
                case THREE_HUB:
                    hand.level3();
                    break;
            }
            return false;
        }
        else
        {
            return true;
        }
    }

    void resetElevator()
    {

        if(!withoutPID())
        {
            dip.releaseFreight();
        }

        elevator.setPower(powerSlowElevator);
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
        spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
        switch (elevatorLevel)
        {
            case 1:
                if(resetElevator.seconds() > 1.3)
                {
                    if(!withoutPID())
                    {
                        hand.intake();
                    }
                    elevator.setUsePID(true);
                }
                else
                {
                    elevator.setUsePID(false);
                }
                break;
            case 2:
                if(resetElevator.seconds() > .55)
                {
                    if(!withoutPID())
                    {
                        hand.intake();
                    }
                    elevator.setUsePID(true);
                }
                else
                {
                    elevator.setUsePID(false);
                }
                break;
            case 3:
                if(resetElevator.seconds() > .25)
                {
                    if(!withoutPID())
                    {
                        hand.intake();
                    }
                    elevator.setUsePID(true);
                }
                else
                {
                    elevator.setUsePID(false);
                }
                break;
        }

        canIntake = true;
    }

}
