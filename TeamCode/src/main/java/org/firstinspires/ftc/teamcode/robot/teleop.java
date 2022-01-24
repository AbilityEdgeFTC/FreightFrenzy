package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.Subsystems.ElevatorThread;
import org.firstinspires.ftc.teamcode.robot.Subsystems.MultitaskingThread;
import org.firstinspires.ftc.teamcode.robot.Subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.Subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.Subsystems.gamepad;

@Config
@TeleOp(group = "main")
public class teleop extends LinearOpMode {

    carousel carousel;
    gamepad gamepads;
    //SampleMecanumDriveCancelable drive;

    @Override
    public void runOpMode() throws InterruptedException {
        //initAll();
        cGamepad cGamepad1 = new cGamepad(gamepad1);
        cGamepad cGamepad2 = new cGamepad(gamepad2);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        carousel = new carousel(hardwareMap);
        gamepads = new gamepad(hardwareMap, gamepad1, gamepad2, telemetry/*, drive*/);

        Thread ElevatorThread = new ElevatorThread(telemetry, hardwareMap, gamepad2);
        Thread MultitaskingThread = new MultitaskingThread(hardwareMap, gamepad2);

        waitForStart();

        ElevatorThread.start();
        MultitaskingThread.start();

        while (opModeIsActive()) {

            cGamepad1.update();
            cGamepad2.update();
            gamepads.update();

//            // TODO: change to gamepad1
//            if(gamepad2.a)
//            {
//                gamepads.lockOnAngle = !gamepads.lockOnAngle;
//            }

            // TODO: change to gamepad2
            if (gamepad2.dpad_right || gamepad2.dpad_left) {
                carousel.spin();
            }
            else {
                carousel.stop();
            }
        }

        ElevatorThread.interrupt();
        MultitaskingThread.interrupt();
    }

    void initAll() {
        //drive = new SampleMecanumDriveCancelable(hardwareMap);
        //drive.setPoseEstimate(currentPose);
        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
