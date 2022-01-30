/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */
package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDriveCancelable;

@Config
public class gamepad {

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
    public static boolean isRegularDrive = true, slowMove = false;
    SampleMecanumDriveCancelable drivetrain;
    cGamepad cGamepad1, cGamepad2;
    Vector2d vectorDrive, vectorTurn, vectorPower;

    // Define 2 states, driver control or alignment control
    enum Mode {
        NORMAL_CONTROL,
        HOLD_ANGLE
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
                getGamepadDirections(true);

                if (cGamepad1.XOnce()) {
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

                if(lockOnAngle)
                {
                    currentMode = Mode.HOLD_ANGLE;
                }
                else
                {
                    currentMode = Mode.NORMAL_CONTROL;
                }

                mFL.setPower(leftPower_f);
                mBL.setPower(leftPower_b);
                mFR.setPower(rightPower_f);
                mBR.setPower(rightPower_b);
                break;
            case HOLD_ANGLE:
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

                if(gamepad1.a)
                {
                   lockOnAngle = !lockOnAngle;
                }

                if(lockOnAngle)
                {
                    currentMode = Mode.HOLD_ANGLE;
                }
                else
                {
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

        double GamepadAng ,currAng = drivetrain.getExternalHeading(), FinalAng = 0;
        double scalar;

        scalar = Math.sqrt(drive * drive + twist * twist);
        GamepadAng = Math.atan(scalar);
        FinalAng = currAng + GamepadAng;
        drive = scalar * Math.sin(FinalAng);
        strafe = scalar * Math.sin(FinalAng);

        leftPower_f = Range.clip(drive + twist + strafe, -power, power);
        leftPower_b = Range.clip(drive + twist - strafe, -power, power);
        rightPower_f = Range.clip(drive - twist - strafe, -power, power);
        rightPower_b = Range.clip(drive - twist + strafe, -power, power);

        /*
        vectorDrive = new Vector2d(drive, strafe);
        vectorDrive.rotated(vectorDrive.angle() + drivetrain.getExternalHeading());

        leftPower_f = Range.clip(vectorDrive.getX() + twist + vectorDrive.getY(), -power, power);
        leftPower_b = Range.clip(vectorDrive.getX() + twist - strafe, -vectorDrive.getY(), power);
        rightPower_f = Range.clip(vectorDrive.getX() - twist - strafe, -vectorDrive.getY(), power);
        rightPower_b = Range.clip(vectorDrive.getX() - twist + strafe, -vectorDrive.getY(), power);
        */

        /*double theta = vectorDrive.angle();

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[0] = Math.sin(theta + Math.PI / 4); // 0: mFL
        wheelSpeeds[1] = Math.sin(theta - Math.PI / 4); // 1: mFR
        wheelSpeeds[2] = Math.sin(theta - Math.PI / 4); // 2: mBL
        wheelSpeeds[3] = Math.sin(theta + Math.PI / 4); // 3: BR

        normalize(wheelSpeeds, vectorDrive.norm());

        wheelSpeeds[0] += twist;
        wheelSpeeds[1] -= twist;
        wheelSpeeds[2] += twist;
        wheelSpeeds[3] -= twist;

        normalize(wheelSpeeds);

        leftPower_f = wheelSpeeds[0] * mainPower;
        leftPower_b = wheelSpeeds[1] * mainPower;
        rightPower_f = wheelSpeeds[2] * mainPower;
        rightPower_b = wheelSpeeds[3] * mainPower;*/


    }

    protected void normalize(double[] wheelSpeeds, double magnitude) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude) * magnitude;
        }
    }

    protected void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude);
            }
        }

    }

    public double GetmFLPower()
    {
        return mFL.getPower();
    }

    public double GetmFRPower()
    {
        return mFR.getPower();
    }

    public double GetmBLPower()
    {
        return mBL.getPower();
    }

    public double GetmBRPower()
    {
        return mBR.getPower();
    }
}