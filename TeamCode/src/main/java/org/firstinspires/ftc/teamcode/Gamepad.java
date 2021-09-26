package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This file contains an gamepad OpMode for the 2021 Ultimate Goal Season, made by Ability Edge.
 */
@TeleOp(name="Gamepad", group="States")
public class Gamepad extends LinearOpMode {

    private DcMotor leftMotor_f = null;
    private DcMotor leftMotor_b = null;
    private DcMotor rightMotor_f = null;
    private DcMotor rightMotor_b = null;
    private double power = .9;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor_f  = hardwareMap.get(DcMotor.class, "mFL");
        leftMotor_b  = hardwareMap.get(DcMotor.class, "mBL");
        rightMotor_f  = hardwareMap.get(DcMotor.class, "mFR");
        rightMotor_b  = hardwareMap.get(DcMotor.class, "mBR");

        // Reverse the motors who are backwards.
        leftMotor_f.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor_b.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor_f.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor_b.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()){

            // Setup a variable for each drive wheel.
            double leftPower_f;
            double leftPower_b;
            double rightPower_f;
            double rightPower_b;

            // Here we declare the gamepad values given from the gamepad.
            // After we use range.clip(which sets a minimum and maximum for the gamepad values.
            // And lastly by the mechanum wheels gamepad formula we give every wheel a value.
            double drive = -gamepad1.left_stick_y; // Remember, this is reversed!
            double strafe = gamepad1.left_stick_x; // Counteract imperfect strafing
            double twist = gamepad1.right_stick_x;

            leftPower_f = Range.clip(drive + strafe + twist, -power, power);
            leftPower_b   = Range.clip(drive - strafe + twist, -power, power);
            rightPower_f = Range.clip(drive - strafe - twist, -power, power);
            rightPower_b   = Range.clip(drive + strafe - twist, -power, power);

            // Send calculated power to wheels
            leftMotor_f.setPower(leftPower_f);
            leftMotor_b.setPower(leftPower_b);
            rightMotor_f.setPower(rightPower_f);
            rightMotor_b.setPower(rightPower_b);

        }
    }
}
