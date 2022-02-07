/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.robot.subsystems.MultitaskingThreadTeleop;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorThread;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.gamepad;

@Config
@TeleOp(group = "main")
public class teleop extends LinearOpMode {

    carousel carousel;
    gamepad gamepad;
    ElevatorThread elevatorThread;
    MultitaskingThreadTeleop multitaskingThreadTeleop;

    /**
     * Function runs once when pressed, and loops while active.
     * main idea is all teleop functions. Elevator, carousel,
     * dip, and more.
     * @throws InterruptedException, thread.sleep
     */
    @Override
    public void runOpMode() throws InterruptedException {
        carousel = new carousel(hardwareMap); // carousel class functions
        gamepad = new gamepad(hardwareMap, gamepad1, gamepad2, telemetry); // teleop(gamepad) class functions
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // dashboard telemetry

        elevatorThread = new ElevatorThread(hardwareMap, telemetry, gamepad2);
        multitaskingThreadTeleop = new MultitaskingThreadTeleop(hardwareMap, telemetry, gamepad1, gamepad2);
        // 2 threads, one for the elevator, and the other for multitasking such as dipping, intake and more
        Thread ElevatorThread = elevatorThread;
        Thread MultitaskingThread = multitaskingThreadTeleop;

        // start the 2 threads
        ElevatorThread.start();
        MultitaskingThread.start();

        // wait till after init
        waitForStart();

        while (opModeIsActive()) {

            // update the gamepads, see if there are new inputs or we need to run functions such as moving the bot
            gamepad.update();

            while (gamepad2.dpad_right)
            {
                carousel.spin(false, true);
            }

            while (gamepad2.dpad_left)
            {

                carousel.spin(true, true);
            }

            if(!gamepad2.dpad_right && !gamepad2.dpad_left)
            {
                carousel.stop();
            }

            telemetry.addData("mBL: ", gamepad.GetmBLPower());
            telemetry.addData("mBR: ", gamepad.GetmBRPower());
            telemetry.addData("mFL: ", gamepad.GetmFLPower());
            telemetry.addData("mFR: ", gamepad.GetmFRPower());
            telemetry.addData("IMU:", gamepad.getIMU());
            telemetry.update();
        }

        // after we exist the opModeIsActive loop, the opmode stops so we have to interrupt the threads and stop them to make the opmode not crash
        ElevatorThread.interrupt();
        MultitaskingThread.interrupt();
    }

}