package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerLibraryPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "Right Red Async", group = "red")
public class AutoRightRedAsync extends LinearOpMode {

    double startPoseRightX = 13;
    double startPoseRightY = -60;
    double startPoseRightH = 90;
    public static double poseEntranceX = 12;
    public static double poseEntranceY = -61.6;
    public static double poseEntranceH = 180;
    public static double poseCollectX = 50;
    public static double poseCollectY = -61.6;
    public static double poseCollectH = 180;
    public static double poseHelpX = 7;
    public static double poseHelpY = -50;
    public static double poseHelpH = 180;
    ElevatorFirstPID elevator;
    //SpinnerFirstPID spinner;
    ElevatorSpinnerLibraryPID spinner;
    hand hand;
    intake intake;
    dip dip;
    boolean canIntake = true;
    public static double powerSlowElevator = .6, powerElevator = 1, powerElevatorFast = 1, turnXmuch = 3;
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
        //spinner = new SpinnerFirstPID(hardwareMap);
        spinner = new ElevatorSpinnerLibraryPID(hardwareMap);
        intake = new intake(hardwareMap);
        hand = new hand(hardwareMap);
        dip = new dip(hardwareMap);

        Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));
        Pose2d poseHelp = new Pose2d(poseHelpX, poseHelpY, Math.toRadians(poseHelpH));
        DriveConstants.setMaxVel(68);

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

                powerElevator = powerElevatorFast;
                elevator.setPower(powerElevatorFast);
                canIntake = false;

                dip.holdFreight();

                switch (elevatorLevel)
                {
                    case 1:
                        //spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                        spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL1);
                        hand.level1();
                        break;
                    case 2:
                        //spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                        spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL2);
                        hand.level2();
                        break;
                    case 3:
                        //spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                        spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
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
                powerElevator = powerSlowElevator;
                dip.releaseFreight();
                elevator.updateAuto();
                spinner.updateAuto();
                canIntake = true;
                elevator.setPower(powerElevator);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
                hand.intake();
                //spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
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
                intake.intakeBackward();
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
                .lineToSplineHeading(new Pose2d(poseEntrance.getX() + 3.5, poseEntrance.getY(), poseEntrance.getHeading()))
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(1.2)
                .addDisplacementMarker(elevetorClose)
                .addTemporalMarker(intakeForward)
                .waitSeconds(0.3)
                .lineToSplineHeading(poseCollect)
                .turn(Math.toRadians(-turnXmuch))
                .turn(Math.toRadians(turnXmuch))
                .lineToSplineHeading(poseEntrance)
                .addSpatialMarker(new Vector2d(39, -63), intakeBackword)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.85)
                .addDisplacementMarker(elevetorClose)
                .addTemporalMarker(intakeForward)
                .waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(poseCollect.getX() + 1.5, poseCollect.getY(), poseCollect.getHeading()))
                .turn(Math.toRadians(-turnXmuch))
                .turn(Math.toRadians(turnXmuch))
                .lineToSplineHeading(poseEntrance)
                .addSpatialMarker(new Vector2d(39, -63), intakeBackword)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.85)
                .addDisplacementMarker(elevetorClose)
                .addTemporalMarker(intakeForward)
                .waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(poseCollect.getX() + 2, poseCollect.getY(), poseCollect.getHeading()))
                .turn(Math.toRadians(-turnXmuch))
                .turn(Math.toRadians(turnXmuch))
                .lineToSplineHeading(poseEntrance)
                .addSpatialMarker(new Vector2d(39, -63), intakeBackword)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.85)
                .addDisplacementMarker(elevetorClose)
                .addTemporalMarker(intakeForward)
                .waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(poseCollect.getX() + 3, poseCollect.getY(), poseCollect.getHeading()))
                .turn(Math.toRadians(-turnXmuch))
                .turn(Math.toRadians(turnXmuch))
                .lineToSplineHeading(poseEntrance)
                .addSpatialMarker(new Vector2d(39, -63), intakeBackword)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.85)
                .addDisplacementMarker(elevetorClose)
                .addTemporalMarker(intakeForward)
                .waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(poseCollect.getX() + 4, poseCollect.getY(), poseCollect.getHeading()))
                .turn(Math.toRadians(-turnXmuch))
                .turn(Math.toRadians(turnXmuch))
                .lineToSplineHeading(poseEntrance)
                .addSpatialMarker(new Vector2d(39, -63), intakeBackword)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.85)
                .addDisplacementMarker(elevetorClose)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseCollect)
                .build();

        dip.getFreight();
        drive.followTrajectorySequenceAsync(main);

        waitForStart();
        //spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
        spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
        spinner.updateAuto();
        elevator.updateAuto();

        while (opModeIsActive())
        {
            drive.update();
            spinner.updateAuto();
            elevator.updateAuto();
        }
    }


}
