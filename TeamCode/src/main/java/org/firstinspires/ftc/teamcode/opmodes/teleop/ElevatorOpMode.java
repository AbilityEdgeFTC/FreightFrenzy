package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorCOMPLEX_UNSTABLE;

//import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorCOMPLEX_UNSTABLE;
/*
 * This is an example of a more complex path to really test the tuning.
 */
@TeleOp(group = "drive")
public class ElevatorOpMode extends LinearOpMode {

    Elevator elevator;
    ElevatorCOMPLEX_UNSTABLE elevatorCOMPLEX_unstable;

    enum Controller
    {
        regular,
        complex
    }

    public static Controller controller = Controller.regular;

    @Override
    public void runOpMode() throws InterruptedException {

        elevator = new Elevator(hardwareMap, gamepad1);
        elevatorCOMPLEX_unstable = new ElevatorCOMPLEX_UNSTABLE(hardwareMap, gamepad1);

        waitForStart();

        while (opModeIsActive())
        {
            switch (controller)
            {
                case regular:
                    elevator.update();
                    break;
                case complex:
                    elevatorCOMPLEX_unstable.update();
                    break;
            }

            telemetry.update();
        }


    }
}
