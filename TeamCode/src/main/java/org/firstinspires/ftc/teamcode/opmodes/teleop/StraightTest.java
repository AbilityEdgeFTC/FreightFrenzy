package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleTankDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@TeleOp(name = "ELEVATOR", group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 20; // in
    public static double threshold = 1; // in

    @Override
    public void runOpMode() throws InterruptedException {

        SampleTankDrive drive = new SampleTankDrive(hardwareMap);

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive())
        {
            Trajectory trajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .forward(DISTANCE-drive.getPoseEstimate().getX())
                    .build();

            drive.followTrajectory(trajectory);
        }

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
