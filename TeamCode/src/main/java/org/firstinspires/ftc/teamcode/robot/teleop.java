package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.subsystems.gamepadSubsystems;
import org.firstinspires.ftc.teamcode.robot.subsystems.intakeSubsystem;

@Config
@TeleOp
public class teleop extends LinearOpMode {

    public static double powerCarousel = 0.325;
    public static double powerElevator = 0.8;
    public static double powerIntake = 1;
    public static int positionLevelOne = 170;
    public static int positionLevelTwo = 250;
    public static int positionLevelThree = 500;

    double mainPower = .7;
    boolean isRegularDrive = true;

    //carouselSubsystem carousel;
    //elevatorSubsystems elevator;
    intakeSubsystem intake;
    gamepadSubsystems gamepads;

    @Override
    public void runOpMode() throws InterruptedException {

        //carousel = new carouselSubsystem(powerCarousel, hw);
        //elevator = new elevatorSubsystems(powerElevator, hw, positionLevelOne, positionLevelTwo, positionLevelThree);
        gamepads = new gamepadSubsystems(gamepad1, gamepad2, mainPower, isRegularDrive, hardwareMap, telemetry);
        intake = new intakeSubsystem(hardwareMap, powerIntake);

        waitForStart();

        while(opModeIsActive())
        {
            gamepads.update();

            // GAMEPAD1 BUMPER
            if (gamepad1.right_bumper || gamepad1.right_bumper) {
                // TODO: ADD SERVO TURNING TO TAKE OUT THE FRIEGHT IN ELEVATOR AND AUTOMATICLLY AFTER RETURN TO ELEVATOR 0
            }

            // BUTTON Y
            if (gamepad2.y) {
                //elevator.goToLevelThree();
            }

            // BUTTON A
            if (gamepad2.a) {
                //elevator.goToLevelOne();
            }

            // BUTTON B
            if (gamepad2.b || gamepad2.x) {
                //elevator.goToLevelTwo();
            }

            // RIGHT TRIGGER
            if (gamepad2.left_bumper)
            {
                intake.intakeBackward();
            }

            // LEFT TRIGGER
            if (gamepad2.right_bumper)
            {
                intake.intakeForward();
            }

            // D-PAD DOWN
            if (gamepad2.dpad_right) {
                //carousel.spinCarouselMotor();
            }

            // D-PAD UP
            if (gamepad2.dpad_left) {
                //carousel.spinCarouselMotor(true);
            }
        }
    }
}