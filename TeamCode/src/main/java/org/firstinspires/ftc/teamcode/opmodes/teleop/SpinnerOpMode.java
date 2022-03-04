package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerLibraryPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerFirstPID;

//import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerCOMPLEX_UNSTABLE;
/*
 * This is an example of a more complex path to really test the tuning.
 */
@TeleOp(group = "drive")
public class SpinnerOpMode extends LinearOpMode {

    SpinnerFirstPID elevator;

    enum Controller
    {
        regular,
        complex
    }

    public static Controller controller = Controller.regular;

    @Override
    public void runOpMode() throws InterruptedException {

        elevator = new SpinnerFirstPID(hardwareMap, gamepad1);

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("POS", elevator.getPosition());
            telemetry.addData("TARGET", elevator.getTarget());
            telemetry.update();
        }
    }
}
