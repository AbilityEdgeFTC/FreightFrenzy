package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorRoadRunner;

//import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorCOMPLEX_UNSTABLE;
/*
 * This is an example of a more complex path to really test the tuning.
 */
@TeleOp(group = "drive")
@Config
public class ElevatorOpModeRR extends LinearOpMode {

    ElevatorRoadRunner elevator;


    @Override
    public void runOpMode() throws InterruptedException {

        elevator = new ElevatorRoadRunner(hardwareMap, gamepad1);

        waitForStart();

        while (opModeIsActive())
        {
            elevator.update();

            telemetry.update();
        }


    }
}
