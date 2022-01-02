package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.robot.subsystems.carouselSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.dippingSubsystem;
import org.firstinspires.ftc.teamcode.robot.subsystems.elevatorSubsystems;
import org.firstinspires.ftc.teamcode.robot.subsystems.gamepadSubsystems;
import org.firstinspires.ftc.teamcode.robot.subsystems.intakeSubsystem;

@Config
@TeleOp(group = "main")
public class teleop extends LinearOpMode {

    DcMotor mC, mE, mFL, mBL, mFR, mBR, mI;
    BNO055IMU imu;
    Servo sD;

    public static double powerCarousel = 0.325;
    public static double powerElevator = 0.8;
    public static double powerIntake = 1;
    public static int positionLevelOne = 170;
    public static int positionLevelTwo = 250;
    public static int positionLevelThree = 500;
    public static double intakePosition = 1, dippingPosition = .6;

    double mainPower = .7;
    boolean isRegularDrive = true;

    carouselSubsystem carousel;
    elevatorSubsystems elevator;
    intakeSubsystem intake;
    gamepadSubsystems gamepads;
    dippingSubsystem dip;

    @Override
    public void runOpMode() throws InterruptedException {

        initAll();

        carousel = new carouselSubsystem(mC, powerCarousel);
        elevator = new elevatorSubsystems(mE, powerElevator, positionLevelOne, positionLevelTwo, positionLevelThree);
        gamepads = new gamepadSubsystems(gamepad1, gamepad2, imu, mFL, mBL, mFR, mBR, mainPower, isRegularDrive, telemetry);
        intake = new intakeSubsystem(mI, powerIntake);
        dip = new dippingSubsystem(sD, intakePosition, dippingPosition);

        waitForStart();

        while(opModeIsActive())
        {
            gamepads.update();

            // GAMEPAD1 BUMPER
            if (gamepad1.right_bumper || gamepad1.right_bumper) {
                dip.releaseFreight();
                wait(1500);
                dip.getFreight();
                elevator.goToZeroPos();
            }

            // TODO: change to gamepad2
            // BUTTON Y
            if (gamepad1.y) {
                elevator.goToLevelThree();
            }

            // TODO: change to gamepad2
            // BUTTON A
            if (gamepad1.a) {
                elevator.goToLevelOne();
            }

            // TODO: change to gamepad2
            // BUTTON B
            if (gamepad1.b || gamepad1.x) {
                elevator.goToLevelTwo();
            }

            // TODO: change to gamepad2
            // RIGHT TRIGGER
            if (gamepad1.left_bumper)
            {
                intake.intakeBackward();
            }

            // TODO: change to gamepad2
            // LEFT TRIGGER
            else if (gamepad1.right_bumper)
            {
                intake.intakeForward();
            }
            else
            {
                intake.stop();
            }

            // TODO: change to gamepad2
            if (gamepad1.dpad_right) {
                carousel.spin();
            }
            // TODO: change to gamepad2
            else if (gamepad1.dpad_left) {
                carousel.spin(true);
            }
            else
            {
                carousel.stop();
            }
        }
    }

    void initAll()
    {
        mFL = hardwareMap.get(DcMotor.class, "mFL");
        mBL = hardwareMap.get(DcMotor.class, "mBL");
        mBR = hardwareMap.get(DcMotor.class, "mBR");
        mFR = hardwareMap.get(DcMotor.class, "mFR");
        mFL.setDirection(DcMotor.Direction.REVERSE);
        mBL.setDirection(DcMotor.Direction.REVERSE);
        mI = hardwareMap.get(DcMotor.class, "mI");
        mI.setDirection(DcMotor.Direction.REVERSE);;
        mC = hardwareMap.get(DcMotor.class, "mC");
        mC.setDirection(DcMotor.Direction.REVERSE);
        mE = hardwareMap.get(DcMotor.class, "mE");
        mE.setDirection(DcMotorSimple.Direction.REVERSE);
        mE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sD = hardwareMap.get(Servo.class, "sE");
        initImu();
    }

    void initImu()
    {

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

    }
}