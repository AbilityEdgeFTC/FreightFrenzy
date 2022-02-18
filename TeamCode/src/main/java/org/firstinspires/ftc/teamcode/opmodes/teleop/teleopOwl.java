/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinner;
import org.firstinspires.ftc.teamcode.robot.subsystems.gamepad;

@Config
@TeleOp(group = "main")
public class teleopOwl extends LinearOpMode {

    gamepad gamepad;
    ElevatorSpinner spinner;
    Elevator elevator;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = new gamepad(hardwareMap, gamepad1, gamepad2, telemetry); // teleop(gamepad) class functions
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // dashboard telemetry
        spinner = new ElevatorSpinner(hardwareMap, gamepad1);
        elevator = new Elevator(hardwareMap, gamepad1);

        // wait till after init
        waitForStart();

        while (opModeIsActive()) {

            // update the gamepads, see if there are new inputs or we need to run functions such as moving the bot
            gamepad.update();
            elevator.update();
            spinner.update();

            telemetry.addData("mBL: ", gamepad.GetmBLPower());
            telemetry.addData("mBR: ", gamepad.GetmBRPower());
            telemetry.addData("mFL: ", gamepad.GetmFLPower());
            telemetry.addData("mFR: ", gamepad.GetmFRPower());
            telemetry.update();
        }

    }

}