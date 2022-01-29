package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.myElevatorThreadAuto;
import org.firstinspires.ftc.teamcode.robot.subsystems.myElevatorThreadAuto.ElevatorSate;
import org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class AutonmousTest2 extends LinearOpMode {

    public static double startPoseRightX = 12;
    public static double startPoseRightY = -64.24;
    public static double startPoseRightH = 0;
    public static double poseHubFrontX = -11.7;
    public static double poseHubFrontY = -45;
    public static double poseHubFrontH = 90;
    public static double poseHubRightX = .3;
    public static double poseHubRightY = -45;
    public static double poseHubRightH = 135;
    public static double poseEntranceX = 12;
    public static double poseEntranceY = -67;
    public static double poseEntranceH = 180;
    public static double poseCollectX = 48.78;
    public static double poseCollectY = -67;
    public static double poseCollectH = 180;
    public static int numOfCycles = 5;
    public static double waitTillOpenElevatorFirst = .5;
    public static double waitTillOpenElevator = 1;
    public static double waitTillCloseElevator = 1;
    public static double waitSecTillTurnOnIntake = 3;
    public static double reverseIntakeFor = 1;
    Thread elevatorThread;
    myElevatorThreadAuto elevator;
    intake intake;
    double delay;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

         Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
         Pose2d poseHubFront = new Pose2d(poseHubFrontX, poseHubFrontY, Math.toRadians(poseHubFrontH));
         Pose2d poseHubRight = new Pose2d(poseHubRightX, poseHubRightY, Math.toRadians(poseHubRightH));
         Pose2d poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
         Pose2d poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));
        drive.setPoseEstimate(startPoseRight);
        elevatorThread = new myElevatorThreadAuto(hardwareMap);
        elevator = new myElevatorThreadAuto(hardwareMap);
        intake= new intake(hardwareMap);
        // "Collect And Place Additional Freight", "Number Of Freight To Collect";
        // "Place Freight In";
        // "Park In";
        // "Park Completely";

        Trajectory placement = drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(poseHubFront, SampleMecanumDrive.getVelocityConstraint(38.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(waitTillOpenElevatorFirst, () -> {
                    elevator.elevatorSate = ElevatorSate.MAX;
                    delay += waitTillOpenElevatorFirst;
                })
                .build();

        delay = 0;

        TrajectorySequence collectAndPlaceFirst = drive.trajectorySequenceBuilder(placement.end())
                .addTemporalMarker(waitTillCloseElevator, () -> {
                    elevator.elevatorSate = ElevatorSate.ZERO;
                    delay += waitTillCloseElevator;
                })
                .lineToLinearHeading(poseEntrance, SampleMecanumDrive.getVelocityConstraint(39, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(delay + waitSecTillTurnOnIntake, () -> {
                    intake.intakeForward();
                    delay += waitSecTillTurnOnIntake;
                })
                .lineToLinearHeading(poseCollect, SampleMecanumDrive.getVelocityConstraint(34.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(delay + reverseIntakeFor, () -> {
                    intake.intakeBackward();
                    delay += reverseIntakeFor;
                })
                .lineToLinearHeading(poseEntrance, SampleMecanumDrive.getVelocityConstraint(39, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(delay + waitTillOpenElevatorFirst, () -> {
                    intake.stop();
                    elevator.elevatorSate = ElevatorSate.MAX;
                    delay += waitTillOpenElevatorFirst;
                })
                .lineToLinearHeading(poseHubRight, SampleMecanumDrive.getVelocityConstraint(38.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        delay = 0;

        TrajectorySequence collectAndPlace = drive.trajectorySequenceBuilder(collectAndPlaceFirst.end())
                .addTemporalMarker(delay + waitTillCloseElevator, () -> {
                    elevator.elevatorSate = ElevatorSate.ZERO;
                    delay += waitTillCloseElevator;
                })
                .lineToLinearHeading(poseEntrance, SampleMecanumDrive.getVelocityConstraint(39, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(delay + waitSecTillTurnOnIntake, () -> {
                    intake.intakeForward();
                    delay += waitSecTillTurnOnIntake;
                })
                .lineToLinearHeading(poseCollect, SampleMecanumDrive.getVelocityConstraint(34.5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(delay + reverseIntakeFor, () -> {
                    intake.intakeBackward();
                    delay += reverseIntakeFor;
                })
                .lineToLinearHeading(poseEntrance, SampleMecanumDrive.getVelocityConstraint(39, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(delay + waitTillOpenElevator, () -> {
                    intake.stop();
                    elevator.elevatorSate = ElevatorSate.MAX;
                    delay += waitTillOpenElevator;
                })
                .lineToLinearHeading(poseHubRight, SampleMecanumDrive.getVelocityConstraint(38.8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        elevatorThread.start();

        waitForStart();

        if (isStopRequested()) return;


        drive.followTrajectory(placement);
        drive.followTrajectorySequence(collectAndPlaceFirst);

        for(int i = 0; i < numOfCycles; i++)
        {
            drive.followTrajectorySequence(collectAndPlace);
        }
        while (opModeIsActive())
        {
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();
            valueStorage.currentPose = poseEstimate;
        }

        elevatorThread.interrupt();


        while (!isStopRequested() && opModeIsActive()) ;
    }
}
