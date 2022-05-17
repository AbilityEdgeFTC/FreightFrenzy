package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDrive;

@Config
@TeleOp(name = "Field Drive Testing", group = "test")
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
        }

    }


}