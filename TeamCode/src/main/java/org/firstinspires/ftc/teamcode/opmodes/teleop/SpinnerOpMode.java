package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@TeleOp(group = "drive")
public class SpinnerOpMode extends LinearOpMode {

    ElevatorSpinner elevator;

    @Override
    public void runOpMode() throws InterruptedException {

        elevator = new ElevatorSpinner(hardwareMap, gamepad1);

        waitForStart();

        while (opModeIsActive())
        {
            elevator.update();

            telemetry.addData("Current", elevator.getPosition());
            //telemetry.addData("Current", elevator.encoderTicksToDegrees(elevator.getPosition()));
            telemetry.addData("Target", elevator.encoderTicksToRadians((int)elevator.getTarget()));
            telemetry.addData("Error", elevator.encoderTicksToRadians((int)elevator.getTarget()) - elevator.encoderTicksToRadians(elevator.getPosition()));
            telemetry.update();
        }


    }
}
