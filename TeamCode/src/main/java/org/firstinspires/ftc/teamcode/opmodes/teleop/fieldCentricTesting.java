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
import org.firstinspires.ftc.teamcode.robot.roadrunner.SampleMecanumDrive;

@Config
@TeleOp(name = "Field Centric Testing", group = "test")
public class fieldCentricTesting extends LinearOpMode {

    DcMotor mFL, mBL, mBR, mFR;
    double leftPower_f = 0, leftPower_b = 0, rightPower_f = 0, rightPower_b = 0;
    double startingHeading = 0;
    double twist = 0;
    public static double power = 0.65;

    @Override
    public void runOpMode() throws InterruptedException {

        mFL = hardwareMap.get(DcMotor.class, "mFL");
        mBL = hardwareMap.get(DcMotor.class, "mBL");
        mBR = hardwareMap.get(DcMotor.class, "mBR");
        mFR = hardwareMap.get(DcMotor.class, "mFR");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        startingHeading = Double.parseDouble(ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("RRheadingValue.txt")));
        drive.setPoseEstimate(new Pose2d(0,0, startingHeading));

        // wait till after init
        waitForStart();

        while (opModeIsActive()) {
            drive.update();

            twist = gamepad1.right_stick_x;

            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x
            ).rotated(drive.getExternalHeading());

            leftPower_f = Range.clip(input.getX() + twist + input.getY() , -power, power);
            leftPower_b = Range.clip(input.getX() + twist - input.getY(), -power, power);
            rightPower_f = Range.clip(input.getX() - twist - input.getY(), -power, power);
            rightPower_b = Range.clip(input.getX() - twist + input.getY(), -power, power);

            mFL.setPower(leftPower_f);
            mBL.setPower(leftPower_b);
            mFR.setPower(rightPower_f);
            mBR.setPower(rightPower_b);

            telemetry.addData("STARTING IMU", startingHeading);
            telemetry.addData("IMU", drive.getRawExternalHeading());
            telemetry.update();

        }

        ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("RRheadingValue.txt"), "" + drive.getRawExternalHeading());

    }


}