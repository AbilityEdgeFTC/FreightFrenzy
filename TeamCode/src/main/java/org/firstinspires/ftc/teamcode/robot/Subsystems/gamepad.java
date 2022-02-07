/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */
package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
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
    public static double startH = 0;

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
    public gamepad(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry) {
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

        try
        {
            startH = Double.parseDouble(ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("RRheadingValue.txt")));
        }catch (NumberFormatException e)
        {
            startH = 0;
        }

        this.drivetrain = new SampleMecanumDriveCancelable(hardwareMap);
        this.drivetrain.setPoseEstimate(new Pose2d(0,0,startH));
        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        this.drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        cGamepad1.update();
        cGamepad2.update();

        switch (currentMode) {
            case NORMAL_CONTROL:
                getGamepadDirections(true);

                if (gamepad1.right_stick_button || gamepad1.left_stick_button) {
                    slowMove = true;
                }
                else
                {
                    slowMove = false;
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
        double theta = (drivetrain.getExternalHeading() - startH) + (Math.PI/2); // when starting in red
        //double theta = (drivetrain.getExternalHeading() - startH) - (Math.PI/2); // when starting in blue

        drive = drive * Math.cos(theta) - strafe * Math.sin(theta);
        strafe = -drive * Math.sin(theta) + strafe * Math.cos(theta);

        leftPower_f = Range.clip(drive + twist + strafe, -power, power);
        leftPower_b = Range.clip(drive + twist - strafe, -power, power);
        rightPower_f = Range.clip(drive - twist - strafe, -power, power);
        rightPower_b = Range.clip(drive - twist + strafe, -power, power);

        telemetry.addData("leftFPower:", drive + twist + strafe);
        telemetry.addData("leftBPower:", drive + twist - strafe);
        telemetry.addData("rightFPower:", drive - twist - strafe);
        telemetry.addData("rightBPower:", drive - twist + strafe);
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

    public double getIMU()
    {
        return drivetrain.getExternalHeading();
    }

}