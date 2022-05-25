package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "Field Drive Testing", group = "test")
@Disabled
public class teleopDrive extends LinearOpMode {

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

        // wait till after init
        waitForStart();

        while (opModeIsActive()) {
            twist = gamepad1.right_stick_x;
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;

            leftPower_f = Range.clip(drive + twist + strafe , -power, power);
            leftPower_b = Range.clip(drive + twist - strafe, -power, power);
            rightPower_f = Range.clip(drive - twist - strafe, -power, power);
            rightPower_b = Range.clip(drive - twist + strafe, -power, power);

            mFL.setPower(leftPower_f);
            mBL.setPower(leftPower_b);
            mFR.setPower(rightPower_f);
            mBR.setPower(rightPower_b);

            telemetry.addData("drive", drive);
            telemetry.addData("twist", twist);
            telemetry.addData("strafe", strafe);
            telemetry.update();
        }

    }


}