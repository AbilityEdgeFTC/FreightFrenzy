package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "Right Red NEW", group = "Autonomous")
public class autoOld extends LinearOpMode {

    double startPoseRightX = 13;
    double startPoseRightY = -60;
    double startPoseRightH = 90;
    public static double poseEntranceX = 13.5;
    public static double poseEntranceY = -67.3;
    public static double poseEntranceH = 180;
    public static double poseCollectX = 51.6;
    public static double poseCollectY = -67.3;
    public static double poseCollectH = 180;
    public static double poseHelpX =9;
    public static double poseHelpY = -52;
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
        Pose2d poseHelp = new Pose2d(poseHelpX, poseHelpY, Math.toRadians(poseHelpH));
        DriveConstants.setMaxVel(60);
        DriveConstants.setMaxVAcc(45);

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
                .lineToSplineHeading(new Pose2d(poseEntrance.getX(), poseEntrance.getY(), poseEntrance.getHeading()))
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.8)
                .addDisplacementMarker(elevetorClose)
                .addTemporalMarker(intakeForward)
                .waitSeconds(0.3)
                .lineToSplineHeading(poseCollect)
                .waitSeconds(1.24)
                .addTemporalMarker(intakeBackword)
                .lineToSplineHeading(poseEntrance)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.8)
                .addDisplacementMarker(elevetorClose)
                .addTemporalMarker(intakeForward)
                .waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(poseCollect.getX() + 4.2, poseCollect.getY() + 0.5, poseCollect.getHeading()), SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(45))
                .waitSeconds(1.24)
                .addTemporalMarker(intakeBackword)
                .lineToSplineHeading(new Pose2d(poseEntrance.getX()-2, poseEntrance.getY() + 0.5, poseEntrance.getHeading()))
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.8)
                .addDisplacementMarker(elevetorClose)
                .addTemporalMarker(intakeForward)
                .waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(poseCollect.getX() + 5.4, poseCollect.getY() + 1, poseCollect.getHeading()), SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(45))
                .waitSeconds(1.24)
                .addTemporalMarker(intakeBackword)
                .lineToSplineHeading(new Pose2d(poseEntrance.getX()-4, poseEntrance.getY() + 1, poseEntrance.getHeading()))
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.8)
                .addDisplacementMarker(elevetorClose)
                .addTemporalMarker(intakeForward)
                .waitSeconds(0.3)
                .lineToSplineHeading(new Pose2d(poseCollect.getX() + 6.8, poseCollect.getY() + 1.5, poseCollect.getHeading()), SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(45))
                .waitSeconds(1.24)
                .addTemporalMarker(intakeBackword)
                .lineToSplineHeading(new Pose2d(poseEntrance.getX()-7.5, poseEntrance.getY() + 1.5, poseEntrance.getHeading()))
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.8)
                .addDisplacementMarker(elevetorClose)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(new Pose2d(poseCollect.getX() + 5, poseCollect.getY() + 2, poseCollect.getHeading()), SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(45))
                .waitSeconds(1.24)
                .addTemporalMarker(intakeBackword)
                .lineToSplineHeading(new Pose2d(poseEntrance.getX()-8.6, poseEntrance.getY() + 2, poseEntrance.getHeading()))
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.8)
                .addDisplacementMarker(elevetorClose)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY() + 4.4, poseCollect.getHeading()), SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(45))
                .build();

        dip.getFreight();

        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.ZERO_RED);

        waitForStart();
        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
        spinner.updateAuto();
        elevator.updateAuto();

        drive.followTrajectorySequence(main);
        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
        spinner.updateAuto();
    }

}