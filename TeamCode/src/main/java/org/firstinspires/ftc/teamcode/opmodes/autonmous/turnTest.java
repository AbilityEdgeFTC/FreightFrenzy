package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
@Autonomous(group = "Tests")
@Disabled
public class turnTest extends LinearOpMode {

    private DcMotor leftMotor_f = null;
    private DcMotor leftMotor_b = null;
    private DcMotor rightMotor_f = null;
    private DcMotor rightMotor_b = null;

    public static double speed = .5;
    public static double angleToTurn = 90;

    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() throws InterruptedException {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double oldAngleToTurn=angleToTurn;
        double scaledSpeed = speed;
        double targetHeading=angles.firstAngle + angleToTurn;
        double oldAngle=angles.firstAngle;

        if(targetHeading<-180)
        {
            targetHeading+=360;
        }
        if(targetHeading>180)
        {
            targetHeading-=360;
        }

        double degreesLeft = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))+(int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);

        while(opModeIsActive() && degreesLeft>1&& oldAngleToTurn-degreesLeft>=0) { //check to see if we overshot target
            scaledSpeed=degreesLeft/(100+degreesLeft)*speed;
            if(scaledSpeed>1)
            {
                scaledSpeed=.1;
            }
            leftMotor_b.setPower(scaledSpeed*1.3);
            rightMotor_b.setPower(scaledSpeed*-1.3);
            leftMotor_f.setPower(scaledSpeed);
            rightMotor_f.setPower(scaledSpeed*-1);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            oldAngleToTurn=degreesLeft;
            degreesLeft = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))+(int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
            if(Math.abs(angles.firstAngle-oldAngle)<1){speed*=1.1;} //bump up speed to wheels in case teleop stalls before reaching target
            oldAngle=angles.firstAngle;

            telemetry.addData("Angle left to turn", angleToTurn);
            telemetry.addData("Angle rn", angles.firstAngle);
        }
        leftMotor_b.setPower(0);
        rightMotor_b.setPower(0);
        leftMotor_f.setPower(0);
        rightMotor_f.setPower(0); //our helper method to set all wheel motors to zero

        sleep(250); //small pause at end of turn
    }
}
