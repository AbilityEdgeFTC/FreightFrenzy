package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.opmodes.Vision.HSVPipeline;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerLibraryPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;

@Config
@Autonomous(name = "Left Red FULL", group = "red")
public class AutoLeftRed extends LinearOpMode {

    public static double startPoseLeftX = -35;
    public static double startPoseLeftY = -60;
    public static double startPoseLeftH = 90;
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
    public static double powerSlowElevator = .6, powerElevator = 1, powerElevatorFast = 1;
    carousel carousel;
    ElevatorFirstPID elevator;
    SpinnerFirstPID spinner;
    hand hand;
    intake intake;
    dip dip;
    //OpenCvWebcam webcam;
    HSVPipeline pipeline;
    SampleMecanumDrive drive;

    //public static boolean withVision = true;

    boolean canIntake = true;

    enum levels
    {
        MIN,
        MID,
        MAX;

    }

    AutoRightRed.levels placeFreightIn = AutoRightRed.levels.MAX;



    public static int elevatorLevel = 3;

    TrajectorySequence main;

    @Override
    public void runOpMode() throws InterruptedException {
        pipeline = new HSVPipeline();
        pipeline.telemetry = telemetry;
        pipeline.DEBUG = false;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        elevator = new ElevatorFirstPID(hardwareMap);
        spinner = new SpinnerFirstPID(hardwareMap);
        //spinner = new ElevatorSpinnerLibraryPID(hardwareMap);
        intake = new intake(hardwareMap);
        hand = new hand(hardwareMap);
        dip = new dip(hardwareMap);
        carousel = new carousel(hardwareMap);

        MarkerCallback elevetorOpen = new MarkerCallback()
        {
            @Override
            public void onMarkerReached() {
                elevator.updateAuto();
                spinner.updateAuto();
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
                        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.LEFT_AUTO_ANGLE);
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.AUTO_LEFT_LEVEL);
                        hand.level3();
                        break;
                }

                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

        MarkerCallback elevetorDuck = new MarkerCallback()
        {
            @Override
            public void onMarkerReached() {
                elevator.updateAuto();
                spinner.updateAuto();

                powerElevator = powerElevatorFast;
                elevator.setPower(powerElevatorFast);
                canIntake = false;

                dip.holdFreight();

                spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.DUCK_ANGLE);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.DUCK_RED_LEVEL);

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

        Pose2d startPoseLeft = new Pose2d(startPoseLeftX, startPoseLeftY, Math.toRadians(startPoseLeftH));
        Pose2d poseCarousel = new Pose2d(poseCarouselX, poseCarouselY, Math.toRadians(poseCarouselH));
        Pose2d poseParkingHelp = new Pose2d(poseParkHelpX,poseParkHelpY,Math.toRadians(poseParkHelpH));
        Pose2d poseParkinga = new Pose2d(poseParkaX, poseParkaY, Math.toRadians(poseParkaH));
        Pose2d poseParkingb = new Pose2d(poseParkbX, poseParkbY, Math.toRadians(poseParkbH));
        Pose2d poseParkingc = new Pose2d(poseParkcX, poseParkcY, Math.toRadians(poseParkcH));

        //initPipeline();

        drive.setPoseEstimate(startPoseLeft);

        main = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(elevetorOpen)
                .waitSeconds(1)
                .addDisplacementMarker(elevetorClose)
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
                .addDisplacementMarker(elevetorDuck)
                .waitSeconds(1)
                .addDisplacementMarker(elevetorClose)
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
        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.LEFT_AUTO_ANGLE);
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
        spinner.updateAuto();
        elevator.updateAuto();

        drive.followTrajectorySequence(main);

        ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("RRheadingValue.txt"), "" + drive.getExternalHeading());
    }

}
