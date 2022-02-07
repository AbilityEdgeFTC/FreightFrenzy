/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorThread;
import org.firstinspires.ftc.teamcode.robot.subsystems.MultitaskingThreadTeleop;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.gamepad;

@Config
@TeleOp(group = "main")
public class teleopOwl extends LinearOpMode {

    gamepad gamepad;

    /**
     * Function runs once when pressed, and loops while active.
     * main idea is all teleop functions. Elevator, carousel,
     * dip, and more.
     * @throws InterruptedException, thread.sleep
     */
    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = new gamepad(hardwareMap, gamepad1, gamepad2, telemetry); // teleop(gamepad) class functions
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // dashboard telemetry

        // wait till after init
        waitForStart();

        while (opModeIsActive()) {

            // update the gamepads, see if there are new inputs or we need to run functions such as moving the bot
            gamepad.update();

            telemetry.addData("mBL: ", gamepad.GetmBLPower());
            telemetry.addData("mBR: ", gamepad.GetmBRPower());
            telemetry.addData("mFL: ", gamepad.GetmFLPower());
            telemetry.addData("mFR: ", gamepad.GetmFRPower());
            telemetry.addData("IMU:", gamepad.getIMU());
            telemetry.update();
        }

    }

}