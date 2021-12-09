package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
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

    public robot(HardwareMap hw) {
        this.hw = hw;
    }

    carouselSubsystem carousel = new carouselSubsystem(powerCarousel, hw);
    elevatorSubsystems elevator = new elevatorSubsystems(powerElevator, hw, positionLevelOne, positionLevelTwo, positionLevelThree);

    public void runCarousel()
    {
        carousel.spinCarouselMotor();
    }

    public void stopCarousel()
    {
        carousel.stopCarouselMotor();
    }

    public void runElevatorLevel0()
    {
        elevator.goToZeroPos();
    }

    public void runElevatorLevel1()
    {
        elevator.goToLevelOne();
    }

    public void runElevatorLevel2()
    {
        elevator.goToLevelTwo();
    }

    public void runElevatorLevel3()
    {
        elevator.goToLevelThree();
    }




}
