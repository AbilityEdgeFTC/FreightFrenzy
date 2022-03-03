package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerMotion;

import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerMotion.LEFT_ANGLE;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerMotion.RIGHT_ANGLE;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerMotion.ZERO_ANGLE;

//import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerCOMPLEX_UNSTABLE;
/*
 * This is an example of a more complex path to really test the tuning.
 */
@TeleOp(group = "drive")
public class SpinnerOpModeMotion extends LinearOpMode {

    ElevatorSpinnerMotion elevator;

    enum Angle
    {
        LEFT,
        ZERO,
        RIGHT
    }

    public static Angle angle = Angle.ZERO;

    @Override
    public void runOpMode() throws InterruptedException {

        elevator = new ElevatorSpinnerMotion(hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {
            switch (angle)
            {
                case LEFT:
                    elevator.setAngle(LEFT_ANGLE);
                    break;
                case ZERO:
                    elevator.setAngle(ZERO_ANGLE);
                    break;
                case RIGHT:
                    elevator.setAngle(RIGHT_ANGLE);
                    break;
            }

            elevator.update();
        }
    }
}
