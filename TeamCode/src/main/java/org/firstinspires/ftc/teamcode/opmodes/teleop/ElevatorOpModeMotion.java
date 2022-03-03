package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorMotionProfile.HUB_LEVEL1;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorMotionProfile.HUB_LEVEL2;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorMotionProfile.HUB_LEVEL3;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorMotionProfile.ZERO_HEIGHT;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorMotionProfile;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorRoadRunner;

//import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorCOMPLEX_UNSTABLE;
/*
 * This is an example of a more complex path to really test the tuning.
 */
@TeleOp(group = "drive")
@Config
public class ElevatorOpModeMotion extends LinearOpMode {

    ElevatorMotionProfile elevator;

    enum Height{
        ONE,
        TWO,
        THREE,
        SHARED,
        ZERO
    }

    public static double timeTo = 5;

    public static Height height = Height.ZERO;

    @Override
    public void runOpMode() throws InterruptedException {

        elevator = new ElevatorMotionProfile(hardwareMap);
        NanoClock clock = NanoClock.system();
        waitForStart();

        while (opModeIsActive())
        {
            switch (height)
            {
                case ZERO:
                    elevator.setHeight(ZERO_HEIGHT);
                    break;
                case ONE:
                    elevator.setHeight(HUB_LEVEL1);
                    break;
                case TWO:
                    elevator.setHeight(HUB_LEVEL2);
                    break;
                case THREE:
                    elevator.setHeight(HUB_LEVEL3);
                    break;

            }

            double startTime = clock.seconds();

            while (!isStopRequested() && (clock.seconds() - startTime) < timeTo) {
                elevator.update();
            }

            elevator.update();

            telemetry.update();
        }


    }
}
