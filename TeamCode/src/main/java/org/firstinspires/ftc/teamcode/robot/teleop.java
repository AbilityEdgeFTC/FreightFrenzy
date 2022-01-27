/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.subsystems.MultitaskingThreadTeleop;
import org.firstinspires.ftc.teamcode.robot.subsystems.MultitaskingThreadTeleop2;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.gamepad;

@Config
@TeleOp(group = "main")
public class teleop extends LinearOpMode {

    carousel carousel;
    gamepad gamepad;

    /**
     * Function runs once when pressed, and loops while active.
     * main idea is all teleop functions. Elevator, carousel,
     * dip, and more.
     * @throws InterruptedException, thread.sleep
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // classes of the gamepad++, functions like gamepad pressed once  and more
        cGamepad cGamepad1 = new cGamepad(gamepad1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // dashboard telemetry
        carousel = new carousel(hardwareMap); // carousel class functions
        gamepad = new gamepad(hardwareMap, gamepad1, telemetry); // teleop(gamepad) class functions

        // 2 threads, one for the elevator, and the other for multitasking such as dipping, intake and more
        Thread MultitaskingThreadTeleop = new MultitaskingThreadTeleop(hardwareMap, gamepad1);
        Thread MultitaskingThreadTeleop2 = new MultitaskingThreadTeleop2(hardwareMap, gamepad1, gamepad2);
        // start the 2 threads
        MultitaskingThreadTeleop.start();
        MultitaskingThreadTeleop2.start();

        // wait till after init
        waitForStart();

        while (opModeIsActive()) {

            // update the gamepads, see if there are new inputs or we need to run functions such as moving the bot
            cGamepad1.update();
            gamepad.update();

            if (gamepad1.dpad_right) {
                carousel.spin(false);
            }
            else if (gamepad1.dpad_left) {
                carousel.spin(true);
            }
            else {
                carousel.stop();
            }

            telemetry.addData("mBL: ", gamepad.GetmBLPower());
            telemetry.addData("mBR: ", gamepad.GetmBRPower());
            telemetry.addData("mFL: ", gamepad.GetmFLPower());
            telemetry.addData("mFR: ", gamepad.GetmFRPower());
            telemetry.update();
        }

        // after we exist the opModeIsActive loop, the opmode stops so we have to interrupt the threads and stop them to make the opmode not crash
        MultitaskingThreadTeleop.interrupt();
        MultitaskingThreadTeleop2.interrupt();
    }



}