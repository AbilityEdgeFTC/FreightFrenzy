package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.localizers.T265Localizer;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TrajectoryTest extends LinearOpMode {

    public static Pose2d startPose = new Pose2d(0,0,0);
    public static Pose2d poseRight = new Pose2d(12,12,0);
    public static Pose2d poseFront = new Pose2d(0,24,0);
    public static Pose2d poseLeft = new Pose2d(-12,12,0);


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(poseRight)
                .waitSeconds(5)
                .lineToSplineHeading(poseFront)
                .waitSeconds(5)
                .lineToSplineHeading(poseLeft)
                .waitSeconds(5)
                .lineToSplineHeading(startPose)
                .build();

        T265Localizer localizer = new T265Localizer(hardwareMap);
        localizer.start(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySequence(trajSeq);

        while (opModeIsActive()){
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();
        }

        localizer.stop();
    }
}
