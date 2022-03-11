package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "Left Blue FULL", group = "red")
public class AutoLeftBlue extends LinearOpMode {

    double startPoseRightX = 13;
    double startPoseRightY = 60;
    double startPoseRightH = 270;
    public static double poseEntranceX = 17.5;
    public static double poseEntranceY = 62;
    public static double poseEntranceH = 180;
    public static double poseCollectX = 50;
    public static double poseCollectY = 62;
    public static double poseCollectH = 180;
    public static double poseHubX = 16.5;
    public static double poseHubY = 61;
    public static double poseHubH = 180;
    public static double poseHelpX = 9;
    public static double poseHelpY = 50;
    public static double poseHelpH = 180;
    ElevatorFirstPID elevator;
    SpinnerFirstPID spinner;
    hand hand;
    intake intake;
    dip dip;
    boolean canIntake = true;
    public static double powerSlowElevator = .6, powerElevator = 1;
    SampleMecanumDrive drive;
    TrajectorySequence main;

    enum levels
    {
        MIN,
        MID,
        MAX
    }

    levels placeFreightIn = levels.MAX;

    public static int elevatorLevel = 3;

    @Override
    public void runOpMode() throws InterruptedException
    {
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
        Pose2d poseHub = new Pose2d(poseHubX, poseHubY, Math.toRadians(poseHubH));
        Pose2d poseHelp = new Pose2d(poseHelpX, poseHelpY, Math.toRadians(poseHelpH));

        MarkerCallback elevetorOpen = new MarkerCallback()
        {
            @Override
            public void onMarkerReached() {
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

                powerElevator = powerSlowElevator;
                elevator.setPower(powerElevator);
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
                canIntake = true;
                hand.intake();
                elevator.setPower(powerSlowElevator);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
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
            }
        };

        MarkerCallback intakeBackword =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                dip.getFreight();
                intake.intakeBackward();
            }
        };

        drive.setPoseEstimate(startPoseRight);

        /*
            SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
            SampleMecanumDrive.getAccelerationConstraint(20))
         */

        main = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(7)
                .lineToLinearHeading(poseHelp)
                .lineToLinearHeading(poseHub)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.8)
                .addDisplacementMarker(elevetorClose)
                .splineToLinearHeading(poseEntrance,poseEntranceH)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseCollect)
                .addTemporalMarker(intakeBackword)
                .lineToSplineHeading(poseEntrance)
                .splineToLinearHeading(poseHub,poseHubH)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.8)
                .addDisplacementMarker(elevetorClose)
                .splineToLinearHeading(poseEntrance,poseEntranceH)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseCollect)
                .addTemporalMarker(intakeBackword)
                .lineToSplineHeading(poseEntrance)
                .splineToLinearHeading(poseHub,poseHubH)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.8)
                .addDisplacementMarker(elevetorClose)
                .splineToLinearHeading(poseEntrance,poseEntranceH)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseCollect)
                .addTemporalMarker(intakeBackword)
                .lineToSplineHeading(poseEntrance)
                .splineToLinearHeading(poseHub,poseHubH)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.8)
                .addDisplacementMarker(elevetorClose)
                .splineToLinearHeading(poseEntrance,poseEntranceH)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseCollect)
                .lineToSplineHeading(poseEntrance)
                .splineToLinearHeading(poseHub,poseHubH)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.8)
                .addDisplacementMarker(elevetorClose)
                .splineToLinearHeading(poseEntrance,poseEntranceH)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseCollect)
                .addTemporalMarker(intakeBackword)
                .lineToSplineHeading(poseEntrance)
                .splineToLinearHeading(poseHub,poseHubH)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.8)
                .addDisplacementMarker(elevetorClose)
                .splineToLinearHeading(poseEntrance,poseEntranceH)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseCollect)
                .build();

        dip.getFreight();

        waitForStart();
        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.ZERO_RED);
        spinner.updateAuto();
        elevator.updateAuto();

        drive.followTrajectorySequence(main);
    }


}
