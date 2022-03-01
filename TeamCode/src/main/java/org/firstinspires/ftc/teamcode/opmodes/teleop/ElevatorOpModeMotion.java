package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorMotion;

import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorMotion.HUB_LEVEL1;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorMotion.HUB_LEVEL2;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorMotion.HUB_LEVEL3;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorMotion.SHARED_HUB;
import static org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorMotion.ZERO_HEIGHT;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@TeleOp(group = "drive")
public class ElevatorOpModeMotion extends LinearOpMode {

    ElevatorMotion elevator;

    enum Height
    {
        ZERO,
        HUB_LEVEL1,
        HUB_LEVEL2,
        HUB_LEVEL3,
        SHARED_HUB
    }

    public static Height height = Height.ZERO;

    @Override
    public void runOpMode() throws InterruptedException {

        elevator = new ElevatorMotion(hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {
            switch (height)
            {
                case ZERO:
                    elevator.setHeight(ZERO_HEIGHT);
                    break;
                case HUB_LEVEL1:
                    elevator.setHeight(HUB_LEVEL1);
                    break;
                case HUB_LEVEL2:
                    elevator.setHeight(HUB_LEVEL2);
                    break;
                case HUB_LEVEL3:
                    elevator.setHeight(HUB_LEVEL3);
                    break;
                case SHARED_HUB:
                    elevator.setHeight(SHARED_HUB);
                    break;
            }

            elevator.update();
        }
    }
}
