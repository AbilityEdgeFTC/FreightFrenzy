package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "Field Drive Encoder Testing", group = "test")
public class DriveEncoderTesting extends LinearOpMode {

    DcMotor mFL, mBL, mBR, mFR;
    double leftPower_f = 0, leftPower_b = 0, rightPower_f = 0, rightPower_b = 0;
    double twist = 0, drive = 0, strafe = 0;
    public static double power = 0.65;

    @Override
    public void runOpMode() throws InterruptedException {
        mFL = hardwareMap.get(DcMotor.class, "mFL");
        mBL = hardwareMap.get(DcMotor.class, "mBL");
        mBR = hardwareMap.get(DcMotor.class, "mBR");
        mFR = hardwareMap.get(DcMotor.class, "mFR");
        this.mFL.setDirection(DcMotor.Direction.REVERSE);
        this.mBL.setDirection(DcMotor.Direction.REVERSE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // wait till after init
        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("LEFT FRONT", mFL.getCurrentPosition());
            telemetry.addData("RIGHT FRONT", mFR.getCurrentPosition());
            telemetry.addData("LEFT BACK", mBL.getCurrentPosition());
            telemetry.addData("RIGHT BACK", mBR.getCurrentPosition());
            telemetry.update();
        }

    }


}