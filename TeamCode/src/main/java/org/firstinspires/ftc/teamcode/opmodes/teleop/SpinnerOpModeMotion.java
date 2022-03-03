package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerMotionProfile.LEFT_ANGLE;
import static org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerMotionProfile.RIGHT_ANGLE;
import static org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerMotionProfile.ZERO_ANGLE;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerMotionProfile;

//import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerCOMPLEX_UNSTABLE;
/*
 * This is an example of a more complex path to really test the tuning.
 */
@TeleOp(group = "drive")
public class SpinnerOpModeMotion extends LinearOpMode {

    SpinnerMotionProfile elevator;

    enum Angle
    {
        LEFT,
        ZERO,
        RIGHT
    }
    public static double timeTo = 5;
    public static Angle angle = Angle.ZERO;

    @Override
    public void runOpMode() throws InterruptedException {

        elevator = new SpinnerMotionProfile(hardwareMap);

        NanoClock clock = NanoClock.system();

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

            double startTime = clock.seconds();

            while (!isStopRequested() && (clock.seconds() - startTime) < timeTo) {
                elevator.update();
            }

            elevator.update();

            elevator.update();
        }
    }
}
