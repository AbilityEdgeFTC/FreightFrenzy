package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives in a DISTANCE-by-DISTANCE square indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin driving in a square.
 * You should observe the target position (green) and your pose estimate (blue) and adjust your
 * follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleOneWheel, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 */
@Config
@Autonomous(group = "drive")
public class goTo0Pos extends LinearOpMode {

    public static double x = 0;
    public static double y = 0;
    public static double heading = 0;

    Pose2d desPose;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(valueStorage.startPoseLeft);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            desPose = new Pose2d(x,y,heading);
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(desPose)
                    .build();

            drive.followTrajectorySequence(trajSeq);
        }
    }
}
