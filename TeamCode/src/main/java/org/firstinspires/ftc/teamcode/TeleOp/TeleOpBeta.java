package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SubSystems.Elevator;


@TeleOp(name = "betateleop" , group = "Teleop")
public class TeleOpBeta extends LinearOpMode
{
    private DcMotor mFL = null;
    private DcMotor mFR = null;
    private DcMotor mBL = null;
    private DcMotor mBR = null;

    Elevator elevator;

    cGamepad m1 = null;

    public enum LiftState {
        LIFT_RETRACT,
        LIFT_BOTTOM,
        LIFT_MID,
        LIFT_MAX,
    };


    @Override
    public void runOpMode() throws InterruptedException
    {
        // Declare our motors
        // Make sure your ID's match your configuration
        mFL = hardwareMap.get(DcMotor.class, "mFL");
        mFR = hardwareMap.get(DcMotor.class, "mFR");
        mBL = hardwareMap.get(DcMotor.class, "mBL");
        mBR = hardwareMap.get(DcMotor.class, "mBR");

        m1 = new cGamepad(gamepad1);

        LiftState liftState = LiftState.LIFT_RETRACT;

        mFL.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.FORWARD);
        mBL.setDirection(DcMotor.Direction.REVERSE);
        mBR.setDirection(DcMotor.Direction.FORWARD);

        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            m1.update();


            switch (liftState) {
                case LIFT_BOTTOM:

                    if(gamepad1.x) {
                        elevator.lowRod(10, 1);
                        liftState = LiftState.LIFT_MID;
                    }else{

                    }
                    break;
                case LIFT_MID:
                    if (gamepad1.b) {
                        elevator.midRod(50, 1);
                        liftState = liftState.LIFT_MAX;
                    }
                    break;
                case LIFT_MAX:
                    if(gamepad1.y){
                        elevator.highRod(100,1);
                        liftState = liftState.LIFT_RETRACT;
                    }
                    break;
                case LIFT_RETRACT:
                    if(gamepad1.a) {
                        elevator.lowRod(0 , -1);
                    }
                break;
                default:
                    liftState = liftState.LIFT_RETRACT;
            }
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double angle = -imu.getAngularOrientation().firstAngle;


            run_Mode_calc(m1.rightBumperOnce());
            if (Globals.run_Mode == 0)
            {
                mechanum(x, y, rx);
            }
            else if(Globals.run_Mode == 1)
            {
                field_centric(x, y, rx, angle);
            }


            telemetry.addData("global heading: ", angle);
            telemetry.addData("x ", x);
            telemetry.addData("y ", y);

            telemetry.addData("m1 ", Globals.run_Mode);
            telemetry.update();
        }
    }

    public int run_Mode_calc(boolean doit)
    {
        if (doit) {
            Globals.run_Mode = Globals.run_Mode + 1;
            Globals.run_Mode = Globals.run_Mode % 2;
        }

        return Globals.run_Mode;
    }

    public static class Globals
    {
        public static int run_Mode = 0;
    }

    public void mechanum(double x, double y, double rx)
    {
        mFL.setPower(y + x + rx);
        mBL.setPower(y - x + rx);
        mFR.setPower(y - x - rx);
        mBR.setPower(y + x - rx);
    }

    public void field_centric(double x, double y, double rx, double angle)
    {
        double rotX = x * Math.cos(angle) - y * Math.sin(angle);
        double rotY = x * Math.sin(angle) + y * Math.cos(angle);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        mFL.setPower((rotY + rotX + rx)/ denominator);
        mBL.setPower((rotY - rotX + rx)/ denominator);
        mFR.setPower((rotY - rotX - rx)/ denominator);
        mBR.setPower((rotY + rotX - rx)/ denominator);
    }

}
