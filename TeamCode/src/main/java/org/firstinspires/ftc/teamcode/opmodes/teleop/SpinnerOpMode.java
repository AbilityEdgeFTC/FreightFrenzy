package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner;

//import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerCOMPLEX_UNSTABLE;
/*
 * This is an example of a more complex path to really test the tuning.
 */
@TeleOp(group = "drive")
public class SpinnerOpMode extends LinearOpMode {

    ElevatorSpinner elevator;
    ElevatorSpinnerCOMPLEX_UNSTABLE elevatorSpinnerCOMPLEX_unstable;

    enum Controller
    {
        regular,
        complex
    }

    public static Controller controller = Controller.regular;

    @Override
    public void runOpMode() throws InterruptedException {

        elevator = new ElevatorSpinner(hardwareMap, gamepad1);
        elevatorSpinnerCOMPLEX_unstable = new ElevatorSpinnerCOMPLEX_UNSTABLE(hardwareMap, gamepad1);

        waitForStart();

        while (opModeIsActive())
        {
            switch (controller)
            {
                case regular:
                    elevator.update();
                    break;
                case complex:
                    elevatorSpinnerCOMPLEX_unstable.update();
                    break;
            }
        }
    }
}
