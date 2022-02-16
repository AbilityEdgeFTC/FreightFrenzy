package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.Elevator;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@TeleOp(group = "drive")
public class ElevatorOpMode extends LinearOpMode {

    Elevator elevator;

    @Override
    public void runOpMode() throws InterruptedException {

        elevator = new Elevator(hardwareMap, gamepad1);

        waitForStart();

        while (opModeIsActive())
        {
            elevator.update();

            telemetry.addData("Current", elevator.getPosition());
            telemetry.addData("Target", elevator.getTarget());
            telemetry.addData("Error", elevator.getTarget() - elevator.getPosition());
            telemetry.update();
        }


    }
}
