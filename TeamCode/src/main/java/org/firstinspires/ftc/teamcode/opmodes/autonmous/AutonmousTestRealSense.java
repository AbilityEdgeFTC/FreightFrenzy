package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.RoadRunner.localizers.DoubleLocalizer;
import org.firstinspires.ftc.teamcode.robot.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class AutonmousTestRealSense extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setLocalizer(new DoubleLocalizer(hardwareMap));

        drive.setPoseEstimate(valueStorage.startPoseLeft);
        //"Spin Carousel";
        // "Collect And Place Additional Freight", "Number Of Freight To Collect";
        // "Place Freight In";
        // "Park In";
        // "Park Completely";

        TrajectorySequence auto = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(valueStorage.poseCarousel1)
                .splineToLinearHeading(valueStorage.poseCarousel2, 0)
                .waitSeconds(1.5)
                .lineToSplineHeading(valueStorage.poseHubFront)
                .waitSeconds(1.5)
                .lineToLinearHeading(valueStorage.poseEntrance)
                .splineToLinearHeading(valueStorage.poseCollect, 0)
                .waitSeconds(1.5)
                .splineToConstantHeading(new Vector2d(valueStorage.poseEntrance.getX(), valueStorage.poseEntrance.getY()), valueStorage.poseEntrance.getHeading())
                .lineToLinearHeading(valueStorage.poseHubFront)
                .waitSeconds(1.5)
                .build();
        /*
        Trajectory collect2 = drive.trajectoryBuilder(placement1.end())
                .lineToSplineHeading(poseEntrance)
                .splineToLinearHeading(poseCollect, drive.getPoseEstimate().getHeading())
                .build();
        Trajectory placement2 = drive.trajectoryBuilder(collect2.end(), true)
                .lineToSplineHeading(poseEntrance)
                .splineToLinearHeading(poseCollect,  Math.toRadians(0))
                .build();
*/
        waitForStart();

        if (isStopRequested()) return;


        drive.followTrajectorySequence(auto);
        /*drive.followTrajectory(collect2);
        Thread.sleep(2000);
        drive.followTrajectory(placement2);
*/
        while (opModeIsActive())
        {
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("finalX", poseEstimate.getX());
            telemetry.addData("finalY", poseEstimate.getY());
            telemetry.addData("finalHeading", poseEstimate.getHeading());
            telemetry.update();
        }


        while (!isStopRequested() && opModeIsActive()) ;
    }
}
