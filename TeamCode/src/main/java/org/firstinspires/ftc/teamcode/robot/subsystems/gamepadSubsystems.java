package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static java.lang.Thread.sleep;

public class gamepadSubsystems{

    imuSubsystems imu;

    HardwareMap hw;
    Telemetry telemetry;

    Gamepad gamepad1, gamepad2;
    DcMotor mFL, mBL, mFR, mBR;

    double power = 0;
    double leftPower_f, leftPower_b, rightPower_f, rightPower_b;
    double drive = 0, strafe = 0, twist = 0;
    double lockAngle = 90;

    boolean regularDrive = true, lockOnAngle = false;

    public gamepadSubsystems(Gamepad gamepad1, Gamepad gamepad2, double power, boolean regularDrive, HardwareMap hw, Telemetry telemetry) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.power = power;
        this.regularDrive = regularDrive;
        this.hw = hw;
        this.telemetry = telemetry;

        imu = new imuSubsystems(hw, telemetry);
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
        double firstAngleRadians = Math.toRadians(imu.angles.firstAngle);

        drive = drive * Math.cos(firstAngleRadians) + strafe * Math.sin(firstAngleRadians);
        strafe = -drive * Math.sin(firstAngleRadians) + strafe * Math.cos(firstAngleRadians);

        leftPower_f = Range.clip(drive + strafe + twist, -power, power);
        leftPower_b = Range.clip(drive - strafe + twist, -power, power);
        rightPower_f = Range.clip(drive - strafe - twist, -power, power);
        rightPower_b = Range.clip(drive + strafe - twist, -power, power);
    }

    public void lockOnAngle(double angle) throws InterruptedException {
        double oldAngleToTurn = angle;
        double scaledSpeed = power;
        double targetHeading = imu.angles.firstAngle + angle;
        double oldAngle = imu.angles.firstAngle;

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

            if(Math.abs(imu.angles.firstAngle-oldAngle)<1){power*=1.1;} //bump up speed to wheels in case teleop stalls before reaching target
            oldAngle = imu.angles.firstAngle;

            telemetry.addData("Angle left to turn", degreesLeft);
            telemetry.addData("Angle rn", imu.angles.firstAngle);
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
        return ((int)(Math.signum(imu.angles.firstAngle - targetHeading)+1)/2)*(360-Math.abs(imu.angles.firstAngle - targetHeading)) +
                (int)(Math.signum(targetHeading-imu.angles.firstAngle)+1)/2*Math.abs(imu.angles.firstAngle-targetHeading);
    }

}
