/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */
package org.firstinspires.ftc.teamcode.robot.subsystems;

//import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
//import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.roadrunner.drive.SampleMecanumDriveCancelable;

@Config
public class gamepad {

    public static boolean DEBUG = true;
    public static boolean redAlliance = false;
    Gamepad gamepad1, gamepad2;
    DcMotor mFL, mBL, mFR, mBR;
    Telemetry telemetry;
    double leftPower_f;
    double leftPower_b;
    double rightPower_f;
    double rightPower_b;
    double drive,  strafe, twist, power = mainPower;
    public static double mainPower = .85, slowPower = .6, multiplier = .9;
    public static boolean slowMove = false, isCentricDrive = true;
    cGamepad cGamepad1, cGamepad2;
    SampleMecanumDriveCancelable drivetrain;
    public static double startH = 0;
    public static boolean holdAngle = false;
    public static double angleToHold = 0;
    //public static String allianceColor;
    //BasicPID pid;
  //  AngleController controller;
    public static double kP = 0.15, kI = 0, kD = 0;
    public static double correction = 5;

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
//        pid = new BasicPID(new PIDCoefficients(kP, kI, kD));
//        controller = new AngleController(pid);
        try
        {
            startH = Double.parseDouble(ReadWriteFile.readFile(AppUtil.getInstance().getSettingsFile("RRheadingValue.txt")));
        }catch (NumberFormatException e)
        {
            startH = 0;
        }

        if(DEBUG && redAlliance)
        {
            startH = 0;
            startH -= Math.PI/2; // red
        }
        else if(DEBUG && !redAlliance)
        {
            startH = 0;
            startH -= Math.PI + Math.PI/2; // blue
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

        if(gamepad1.right_bumper)
        {
            holdAngle = true;
        }
        else if(gamepad1.left_bumper)
        {
            holdAngle = false;
        }

        if(cGamepad1.XOnce())
        {
            isCentricDrive = !isCentricDrive;
        }

        if (isCentricDrive)
        {
            centricDrive();
        }
        else
        {
            regularDrive();
        }

        if(holdAngle)
        {
            getGamepadDirections(false);
            holdAngle();
        }

        mFL.setPower(leftPower_f);
        mBL.setPower(leftPower_b);
        mFR.setPower(rightPower_f);
        mBR.setPower(rightPower_b);
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
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x
        ).rotated(drivetrain.getExternalHeading());

        leftPower_f = Range.clip(input.getX() + twist + input.getY() , -power, power);
        leftPower_b = Range.clip(input.getX() + twist - input.getY(), -power, power);
        rightPower_f = Range.clip(input.getX() - twist - input.getY(), -power, power);
        rightPower_b = Range.clip(input.getX() - twist + input.getY(), -power, power);
    }

    public void holdAngle()
    {
        if(Math.abs(angleToHold - drivetrain.getExternalHeading()) <= Math.toRadians(correction) && Math.abs(angleToHold - drivetrain.getExternalHeading()) >= Math.toRadians(-correction))
        {
            twist = 0;
        }
        else
        {
 //           twist = controller.calculate(Math.toRadians(angleToHold), drivetrain.getExternalHeading());
        }

        leftPower_f = Range.clip(drive + twist + strafe, -power, power);
        leftPower_b = Range.clip(drive + twist - strafe, -power, power);
        rightPower_f = Range.clip(drive - twist - strafe, -power, power);
        rightPower_b = Range.clip(drive - twist + strafe, -power, power);
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

    public void saveIMUHeading()
    {
        ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("RRheadingValue.txt"), "" + getIMU());
    }
}