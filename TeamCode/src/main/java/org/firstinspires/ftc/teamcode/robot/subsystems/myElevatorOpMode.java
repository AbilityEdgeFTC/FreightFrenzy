package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.MecanumLocalizer;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@TeleOp(group = "drive")
public class myElevatorOpMode extends LinearOpMode {

    myElevator elevator;


    @Override
    public void runOpMode() throws InterruptedException {

        elevator = new myElevator(hardwareMap, gamepad1);

        ElapsedTime time = new ElapsedTime();

        waitForStart();

        time.reset();

        while (opModeIsActive())
        {
            elevator.update(time);

            //telemetry.addData("Target", myElevator);
            //telemetry.addData("Error", );
            telemetry.update();
        }


    }
}
