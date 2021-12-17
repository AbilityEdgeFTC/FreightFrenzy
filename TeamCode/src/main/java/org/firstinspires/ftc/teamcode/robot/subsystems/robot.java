package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class robot {

    public static double powerCarousel = 0.325;
    public static double powerElevator = 0.8;
    public static int positionLevelOne = 170;
    public static int positionLevelTwo = 250;
    public static int positionLevelThree = 500;

    HardwareMap hw;
    Telemetry telemetry;
    Gamepad gamepad1, gamepad2;

    double mainPower = .7;
    boolean isRegularDrive = true;

    public robot(HardwareMap hw, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.hw = hw;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    carouselSubsystem carousel = new carouselSubsystem(powerCarousel, hw);
    elevatorSubsystems elevator = new elevatorSubsystems(powerElevator, hw, positionLevelOne, positionLevelTwo, positionLevelThree);
    gamepadSubsystems gamepads = new gamepadSubsystems(gamepad1, gamepad2, mainPower, isRegularDrive, hw, telemetry);

    public void update() throws InterruptedException {

        gamepads.update();

        // BUTTON Y
        if (gamepad1.y) {

        }

        // BUTTON A
        if (gamepad1.a) {

        }

        // BUTTON B
        if (gamepad1.b) {

        }

        // D-PAD DOWN
        if (gamepad1.dpad_down) {

        }

        // D-PAD UP
        if (gamepad1.dpad_up) {

        }

        // RIGHT BUMPER
        if (gamepad1.right_bumper) {

        }

        // LEFT BUMPER
        if (gamepad1.left_bumper) {

        }

        // BUTTON Y
        if (gamepad2.y) {

        }

        // BUTTON A
        if (gamepad2.a) {

        }

        // BUTTON B
        if (gamepad2.b) {

        }

        // D-PAD DOWN
        if (gamepad2.dpad_down) {

        }

        // D-PAD UP
        if (gamepad2.dpad_up) {

        }

        // RIGHT BUMPER
        if (gamepad2.right_bumper) {

        }

        // LEFT BUMPER
        if (gamepad2.left_bumper) {

        }
    }


}
