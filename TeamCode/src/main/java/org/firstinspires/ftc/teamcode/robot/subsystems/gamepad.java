package org.firstinspires.ftc.teamcode.robot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.RoadRunner.drive.SampleMecanumDriveCancelable;
import org.opencv.core.Mat;

import static java.lang.Thread.sleep;

@Config
public class gamepad {

    Orientation angles;
    Gamepad gamepad1, gamepad2;
    DcMotor mFL, mBL, mFR, mBR;
    Telemetry telemetry;
    public double power;
    double leftPower_f;
    double leftPower_b;
    double rightPower_f;
    double rightPower_b;
    double drive;
    double strafe;
    double twist;
    public double lockAngle;
    public boolean regularDrive = false, lockOnAngle = false;
    SampleMecanumDriveCancelable drivetrain;
    public static double multiplier = .5;

    // Define 2 states, driver control or alignment control
    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_ANGLE
    }

    private Mode currentMode = Mode.NORMAL_CONTROL;
    // Declare a PIDF cGamepad to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    //private PIDFController headingController = new PIDFController(SampleMecanumDriveCancelable.HEADING_PID);

    /**
     * constructor for gamepad
     * @param gamepad1 the gamepad1 object from teleop
     * @param gamepad2 the gamepad2 object from teleop
     * @param mFL the mFL object already referenced from HW in teleop
     * @param mBL the mBL object already referenced from HW in teleop
     * @param mFR the mFR object already referenced from HW in teleop
     * @param mBR the mBR object already referenced from HW in teleop
     * @param power the power to give the motors
     * @param regularDrive use regular or centric drive
     * @param telemetry the telemetry object from teleop
     * @param drivetrain the SampleMecanumDriveCancable object from teleop
     * @param power the angle to lock on during teleop
     */
    public gamepad(Gamepad gamepad1, Gamepad gamepad2, DcMotor mFL, DcMotor mBL, DcMotor mFR, DcMotor mBR, double power, boolean regularDrive, Telemetry telemetry, SampleMecanumDriveCancelable drivetrain, double lockAngle) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.mFL = mFL;
        this.mBL = mBL;
        this.mFR = mFR;
        this.mBR = mBR;
        this.power = power;
        this.regularDrive = regularDrive;
        this.telemetry = telemetry;
        this.drivetrain = drivetrain;
        this.lockAngle = lockAngle;

        // Set input bounds for the heading controller
        // Automatically handles overflow
        //headingController.setInputBounds(-Math.PI, Math.PI);
    }

    public void update() {
        switch (currentMode)
        {
            case NORMAL_CONTROL:
                drivetrain.cancelFollowing();
                getGamepadDirections(true);

                regularDrive();

//                if(gamepad1.a)
//                {
//                    lockOnAngle = !lockOnAngle;
//                }
//
//                if(gamepad1.a && lockOnAngle)
//                {
//                    currentMode = Mode.ALIGN_TO_ANGLE;
//                }

                mFL.setPower(leftPower_f);
                mBL.setPower(leftPower_b);
                mFR.setPower(rightPower_f);
                mBR.setPower(rightPower_b);
                break;
            case ALIGN_TO_ANGLE:
                drivetrain.turnAsync(Angle.normDelta(Math.toRadians(lockAngle) - drivetrain.getExternalHeading()));

                getGamepadDirections(false);

                regularDrive();

                if(gamepad1.a)
                {
                    lockOnAngle = !lockOnAngle;
                }

                if(gamepad1.a && !lockOnAngle)
                {
                    lockOnAngle = !lockOnAngle;
                    currentMode = Mode.NORMAL_CONTROL;
                }
                mFL.setPower(leftPower_f);
                mBL.setPower(leftPower_b);
                mFR.setPower(rightPower_f);
                mBR.setPower(rightPower_b);
                break;
        }
    }

    public void getGamepadDirections(boolean canTurn)
    {
        if(canTurn)
        {
            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            twist = gamepad1.right_stick_x * multiplier;
        }
        else
        {
            drive = gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            twist = 0;
        }
    }

    public void regularDrive()
    {
        leftPower_f = Range.clip(drive + twist + strafe, -power, power);
        leftPower_b = Range.clip(drive + twist - strafe, -power, power);
        rightPower_f = Range.clip(drive - twist - strafe, -power, power);
        rightPower_b = Range.clip(drive - twist + strafe, -power, power);
    }


    public boolean isStoped()
    {
        if(leftPower_f == 0 && leftPower_b == 0 && rightPower_f == 0 && rightPower_b == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }


}