package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerPID;

//import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerCOMPLEX_UNSTABLE;
/*
 * This is an example of a more complex path to really test the tuning.
 */
@Config
@TeleOp(name = "Elevator Spinner Testing", group = "testing")
public class SpinnerOpMode extends LinearOpMode {

    SpinnerPID elevator;

    public static boolean usePID = false;

    @Override
    public void runOpMode() throws InterruptedException {

        elevator = new SpinnerPID(hardwareMap, gamepad1, gamepad2);

        waitForStart();

        while (opModeIsActive())
        {
            if(usePID)
            {
                elevator.update();
            }
            else
            {
                elevator.setUsePID(false);
            }

            telemetry.addData("POS", elevator.getPosition());
            telemetry.update();
        }
    }
}
