package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Thread.sleep;

public class gamepadSubsystems{

    // State used for updating telemetry
    Orientation angles;

    Gamepad gamepad1, gamepad2;
    BNO055IMU imu;

    DcMotor mFL, mBL, mFR, mBR;
    Telemetry telemetry;

    double power = 0;
    double leftPower_f, leftPower_b, rightPower_f, rightPower_b;
    double drive = 0, strafe = 0, twist = 0;
    double lockAngle = 90;

    boolean regularDrive = true, lockOnAngle = false;

    public gamepadSubsystems(Gamepad gamepad1, Gamepad gamepad2, BNO055IMU imu, DcMotor mFL, DcMotor mBL, DcMotor mFR, DcMotor mBR, double power, boolean regularDrive, Telemetry telemetry) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.imu = imu;
        this.mFL = mFL;
        this.mBL = mBL;
        this.mFR = mFR;
        this.mBR = mBR;
        this.power = power;
        this.regularDrive = regularDrive;
        this.telemetry = telemetry;

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public void update() throws InterruptedException {
        getGamepadDirections();

        if(regularDrive)
        {
            regularDrive();
        }
        else
        {
            centicDrive();
        }

        if(isStoped() && lockOnAngle)
        {
            lockOnAngle(lockAngle);
        }

        mFL.setPower(leftPower_f);
        mBL.setPower(leftPower_b);
        mFR.setPower(rightPower_f);
        mBR.setPower(rightPower_b);
    }

    public void getGamepadDirections()
    {
        drive = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        twist = gamepad1.right_stick_x;

    }

    public void regularDrive()
    {
        leftPower_f = Range.clip(drive + strafe + twist, -power, power);
        leftPower_b = Range.clip(drive - strafe + twist, -power, power);
        rightPower_f = Range.clip(drive - strafe - twist, -power, power);
        rightPower_b = Range.clip(drive + strafe - twist, -power, power);
    }

    public void centicDrive()
    {
        double firstAngleRadians = Math.toRadians(angles.firstAngle);

        drive = drive * Math.cos(firstAngleRadians) + strafe * Math.sin(firstAngleRadians);
        strafe = -drive * Math.sin(firstAngleRadians) + strafe * Math.cos(firstAngleRadians);

        leftPower_f = Range.clip(drive + strafe + twist, -power, power);
        leftPower_b = Range.clip(drive - strafe + twist, -power, power);
        rightPower_f = Range.clip(drive - strafe - twist, -power, power);
        rightPower_b = Range.clip(drive + strafe - twist, -power, power);
    }

    public void lockOnAngle(double angle) throws InterruptedException {
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double oldAngleToTurn = angle;
        double scaledSpeed = power;
        double targetHeading = angles.firstAngle + angle;
        double oldAngle = angles.firstAngle;

        if(targetHeading<-180)
        {
            targetHeading+=360;
        }
        if(targetHeading>180)
        {
            targetHeading-=360;
        }

        double degreesLeft = calculateDegressLeft(targetHeading);

        while(degreesLeft>1&& oldAngleToTurn-degreesLeft>=0) { //check to see if we overshot target
            scaledSpeed=degreesLeft/(100+degreesLeft)*power;
            if(scaledSpeed>1)
            {
                scaledSpeed=.1;
            }

            leftPower_b = scaledSpeed*1.3;
            rightPower_b = scaledSpeed*-1.3;
            leftPower_f = scaledSpeed;
            rightPower_f = scaledSpeed*-1;

            oldAngleToTurn = degreesLeft;

            degreesLeft = calculateDegressLeft(targetHeading);

            if(Math.abs(angles.firstAngle-oldAngle)<1){power*=1.1;} //bump up speed to wheels in case teleop stalls before reaching target
            oldAngle = angles.firstAngle;

            telemetry.addData("Angle left to turn", degreesLeft);
            telemetry.addData("Angle rn", angles.firstAngle);
        }
        leftPower_b = 0;
        rightPower_b = 0;
        leftPower_f = 0;
        rightPower_f = 0; //our helper method to set all wheel motors to zero

        sleep(250); //small pause at end of turn
    }

    public boolean isStoped()
    {
        if(leftPower_f == 0 && leftPower_b == 0 && rightPower_f == 0 && rightPower_b == 0)
        {
            return true;
        }else
        {
            return false;
        }
    }

    public double calculateDegressLeft(double targetHeading)
    {
        return ((int)(Math.signum(angles.firstAngle - targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle - targetHeading)) +
                (int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
    }

}