/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */
package org.firstinspires.ftc.teamcode.robot.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.RoadRunner.drive.SampleMecanumDriveCancelable;

@Config
public class gamepad {

    Orientation angles;
    Gamepad gamepad1, gamepad2;
    DcMotor mFL, mBL, mFR, mBR;
    Telemetry telemetry;
    double leftPower_f;
    double leftPower_b;
    double rightPower_f;
    double rightPower_b;
    double drive,  strafe, twist, power = mainPower;
    public static double lockAngle = 90;
    public boolean  lockOnAngle = false;
    public static double mainPower = 1, slowPower = .6, multiplier = .9;
    public static boolean isRegularDrive = false, slowMove = false;
    SampleMecanumDriveCancelable drivetrain;
    cGamepad cGamepad1, cGamepad2;
    Vector2d vectorDrive;

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
     * @param telemetry the telemetry object from teleop
     * //@param drivetrain the SampleMecanumDriveCancable object from teleop
     */
    public gamepad(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, SampleMecanumDriveCancelable drivetrain) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.mFL = hardwareMap.get(DcMotor.class, "mFL");
        this.mBL = hardwareMap.get(DcMotor.class, "mBL");
        this.mBR = hardwareMap.get(DcMotor.class, "mBR");
        this.mFR = hardwareMap.get(DcMotor.class, "mFR");
        this.mFL.setDirection(DcMotor.Direction.REVERSE);
        this.mBL.setDirection(DcMotor.Direction.REVERSE);
        cGamepad1 = new cGamepad(gamepad1);
        cGamepad2 = new cGamepad(gamepad2);
        this.telemetry = telemetry;
        mBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.drivetrain = drivetrain;
    }

    public void update() {
        cGamepad1.update();
        cGamepad2.update();

        switch (currentMode) {
            case NORMAL_CONTROL:
                //drivetrain.cancelFollowing();
                getGamepadDirections(true);

                if (cGamepad1.leftBumperOnce() || cGamepad1.rightBumperOnce()) {
                    slowMove = !slowMove;
                }

                if (slowMove) {
                    power = slowPower;
                } else {
                    power = mainPower;
                }

                if (isRegularDrive)
                {
                    regularDrive();
                }
                else
                {
                    centricDrive();
                }


//              if(gamepad1.a)
//              {
//                  lockOnAngle = !lockOnAngle;
//              }
//
//              if(gamepad1.a && lockOnAngle)
//              {
//                  currentMode = Mode.ALIGN_TO_ANGLE;
//              }

                mFL.setPower(leftPower_f);
                mBL.setPower(leftPower_b);
                mFR.setPower(rightPower_f);
                mBR.setPower(rightPower_b);
                break;
            case ALIGN_TO_ANGLE:
                //drivetrain.turnAsync(Angle.normDelta(Math.toRadians(lockAngle) - drivetrain.getExternalHeading()));

                getGamepadDirections(false);

                if (slowMove) {
                    power = slowPower;
                } else {
                    power = mainPower;
                }

                if (isRegularDrive)
                {
                    regularDrive();
                }
                else
                {
                    centricDrive();
                }

//              if(gamepad1.a)
//              {
//                  lockOnAngle = !lockOnAngle;
//              }
//
//              if(gamepad1.a && lockOnAngle)
//              {
//                  currentMode = Mode.ALIGN_TO_ANGLE;
//              }

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
            drive = -gamepad1.left_stick_y;
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

    public void centricDrive()
    {
        vectorDrive = new Vector2d(drive, strafe);
        vectorDrive.rotated(drivetrain.getPoseEstimate().getHeading());
        //vectorDrive.rotated(-drivetrain.getPoseEstimate().getHeading());

        leftPower_f = Range.clip(vectorDrive.getX() + vectorDrive.angle() + vectorDrive.getY(), -power, power);
        leftPower_b = Range.clip(vectorDrive.getX() + vectorDrive.angle() - vectorDrive.getY(), -power, power);
        rightPower_f = Range.clip(vectorDrive.getX() - vectorDrive.angle() - vectorDrive.getY(), -power, power);
        rightPower_b = Range.clip(vectorDrive.getX() - vectorDrive.angle() + vectorDrive.getY(), -power, power);
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