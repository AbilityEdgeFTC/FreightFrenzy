package org.firstinspires.ftc.teamcode.robot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MultitaskingThread extends Thread {

    public static double intakePosition = 1, dippingPosition = .6;
    intake intake;
    Gamepad gamepad1;
    DcMotor mI;
    dip dip;
    Servo sD;

    public MultitaskingThread(Telemetry telemetry, HardwareMap hw, Gamepad gamepad1) throws InterruptedException {
        telemetry.addData("Thread Called: ", this.getName());
        mI = hw.get(DcMotor.class, "mI");
        mI.setDirection(DcMotor.Direction.REVERSE);
        intake = new intake(mI);
        sD = hw.get(Servo.class, "sE");
        dip = new dip(sD, intakePosition, dippingPosition);
        dip.getFreight();
        this.gamepad1 = gamepad1;
    }

    // called when tread.start is called. thread stays in loop to do what it does until exit is
    // signaled by main code calling thread.interrupt.
    @Override
    public void run() {
        try {
            while (!isInterrupted()) {
                // TODO: change to gamepad2
                if (gamepad1.left_trigger != 0) {
                    intake.powerIntake(-gamepad1.left_trigger);
                }
                // TODO: change to gamepad2
                else if (gamepad1.right_trigger != 0) {
                    intake.powerIntake(gamepad1.right_trigger);
                } else {
                    intake.stop();
                }

                // TODO: change to gamepad2
                if(gamepad1.dpad_down)
                {
                    dip.releaseFreight();
                }
                else if(gamepad1.dpad_up)
                {
                    dip.getFreight();
                }
            }
        }
        // an error occurred in the run loop.
        catch (Exception e) {
        }
    }

}