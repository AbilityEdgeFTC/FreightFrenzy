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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.RoadRunner.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.robot.Subsystems.ElevatorThread;
import org.firstinspires.ftc.teamcode.robot.Subsystems.myElevatorThread;
import org.firstinspires.ftc.teamcode.robot.Subsystems.MultitaskingThreadTeleop;
import org.firstinspires.ftc.teamcode.robot.Subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.Subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.Subsystems.gamepad;
import org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage;

@Config
@TeleOp(group = "main")
public class teleop extends LinearOpMode {

    carousel carousel;
    gamepad gamepad;
    SampleMecanumDriveCancelable drive;

    /**
     * Function runs once when pressed, and loops while active.
     * main idea is all teleop functions. Elevator, carousel,
     * dip, and more.
     * @throws InterruptedException, thread.sleep
     */
    @Override
    public void runOpMode() throws InterruptedException {
        initAll();
        // classes of the gamepad++, functions like gamepad pressed once  and more
        cGamepad cGamepad1 = new cGamepad(gamepad1);
        cGamepad cGamepad2 = new cGamepad(gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // dashboard telemetry
        carousel = new carousel(hardwareMap); // carousel class functions
        gamepad = new gamepad(hardwareMap, gamepad1, gamepad2, telemetry, drive); // teleop(gamepad) class functions

        // 2 threads, one for the elevator, and the other for multitasking such as dipping, intake and more
        //Thread myElevatorThread = new myElevatorThread(hardwareMap, gamepad2);
        Thread ElevatorThread = new ElevatorThread(hardwareMap, gamepad2);
        //Thread MultitaskingThread = new MultitaskingThreadTeleop(hardwareMap, gamepad1, gamepad2);

        // start the 2 threads
        //myElevatorThread.start();
        ElevatorThread.start();
        //MultitaskingThread.start();

        // wait till after init
        waitForStart();

        while (opModeIsActive()) {

            // update the gamepads, see if there are new inputs or we need to run functions such as moving the bot
            cGamepad1.update();
            cGamepad2.update();
            gamepad.update();

            if (gamepad2.dpad_right) {
                carousel.spin();
            }
            else if (gamepad2.dpad_left) {
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
        ElevatorThread.interrupt();
       //myElevatorThread.interrupt();
        //MultitaskingThread.interrupt();
    }

    void initAll() {
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        drive.setPoseEstimate(valueStorage.currentPose);
        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



}