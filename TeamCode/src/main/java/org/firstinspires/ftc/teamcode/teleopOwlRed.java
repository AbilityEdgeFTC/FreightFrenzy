/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorSpinnerLibraryPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.gamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;

@Config
@TeleOp(name = "TeleOp Red Alliance", group = "main")
public class teleopOwlRed extends LinearOpMode {

    gamepad gamepad;
    ElevatorSpinnerLibraryPID spinner;
    ElevatorFirstPID elevator;
    carousel carousel;
    intake intake;
    hand hand;
    dip dip;
    private boolean frontIntake = false, backIntake = false;
    cGamepad cGamepad1, cGamepad2;
    public static double powerIntake = 1, powerSlowElevator = .6, powerElevator = 1;
    boolean canIntake = true;
    double position = 0;
    ElapsedTime resetElevator;

    enum ElevatorMovement
    {
        SPIN,
        LEVEL1,
        LEVEL2,
        LEVEL3,
        SHARED,
        DIP
    }

    public static int elevatorLevel = 3;

    public static ElevatorMovement elevatorMovement = ElevatorMovement.SPIN;
    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = new gamepad(hardwareMap, gamepad1, gamepad2, telemetry); // teleop(gamepad) class functions
        gamepad.setRedAlliance(true);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // dashboard telemetry
        spinner = new ElevatorSpinnerLibraryPID(hardwareMap, gamepad1, gamepad2);
        elevator = new ElevatorFirstPID(hardwareMap, gamepad2);
        carousel = new carousel(hardwareMap);
        intake = new intake(hardwareMap);
        hand = new hand(hardwareMap);
        dip = new dip(hardwareMap);
        cGamepad1 = new cGamepad(gamepad1);
        cGamepad2 = new cGamepad(gamepad2);
        resetElevator = new ElapsedTime();
        spinner.setZERO_ANGLE_RED(spinner.getPosition());
        spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.ZERO_RED);
        spinner.update();

        // wait till after init
        waitForStart();


        while (opModeIsActive()) {
            cGamepad1.update();
            cGamepad2.update();
            gamepad.update();
            elevator.update();
            spinner.update();
            intakeToggles();
            elevatorSwitch();
            resetElevatorMidMoving();
            pidToggles();
            carouselSpinning();
            if(withoutPID())
            {
                handMoving();
            }
            if(gamepad1.left_stick_button)
            {
                gamepad.setPower(1);
            }
            else
            {
                gamepad.setPower(gamepad.mainPower);
            }
            elevatorLeveling();
            telemetry.update();
        }

        gamepad.saveIMUHeading();
    }

    void elevatorLeveling()
    {
        if(gamepad2.a)
        {
            elevatorLevel = 1;
        }
        else if(gamepad2.b)
        {
            elevatorLevel = 2;
        }
        else if(gamepad2.y)
        {
            elevatorLevel = 3;
        }
        else if(gamepad2.x)
        {
            elevatorLevel = 0;
        }
    }

    void handMoving()
    {
        position = hand.getPos();

        if(cGamepad2.rightBumperOnce() && hand.getPos() <= 0.99)
        {
            position -= 0.01;
            hand.moveTo(position);
        }
        else if(cGamepad2.leftBumperOnce() && hand.getPos() > 0.01)
        {
            position += 0.01;
            hand.moveTo(position);
        }

    }

    boolean pidToggles()
    {
        if(gamepad2.left_stick_button || gamepad2.right_stick_y != 0 || gamepad2.right_stick_x != 0 || gamepad2.left_stick_y != 0 || gamepad2.left_stick_x != 0 && !gamepad2.right_stick_button)
        {
            spinner.setUsePID(false);
            elevator.setUsePID(false);
            return false;
        }
        else if(gamepad2.right_stick_button || gamepad1.left_bumper && !gamepad2.left_stick_button && elevatorMovement != ElevatorMovement.SPIN)
        {
            spinner.setUsePID(true);
            elevator.setUsePID(true);
        }

        return true;
    }

    void carouselSpinning()
    {
        if(gamepad2.dpad_right)
        {
            carousel.spin(false, true);
        }
        else if(gamepad2.dpad_left)
        {
            carousel.spin(true, true);
        }
        else
        {
            carousel.stop();
        }
    }

    void intakeToggles()
    {
        if ((gamepad1.right_trigger != 0) && canIntake && (gamepad2.right_trigger == 0) && (gamepad2.left_trigger == 0) && (gamepad1.left_trigger == 0)/* && !intakeFixingThread.withFreight()*/)
        {
            intake.powerIntake(gamepad1.right_trigger);
            frontIntake = false;
            backIntake = false;
            resetSpinner();
        }
        else if ((gamepad1.left_trigger != 0) && canIntake && (gamepad2.right_trigger == 0) && (gamepad1.right_trigger == 0) && (gamepad2.left_trigger == 0)/* && !intakeFixingThread.withFreight()*/)
        {
            intake.powerIntake(-gamepad1.left_trigger);
            frontIntake = false;
            backIntake = false;
            resetSpinner();
        }
        else if ((gamepad2.right_trigger != 0) && canIntake && (gamepad2.left_trigger == 0) && (gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0))
        {
            intake.powerIntake(gamepad2.right_trigger);
            frontIntake = false;
            backIntake = false;
            resetSpinner();
        }
        else if ((gamepad2.left_trigger != 0) && canIntake && (gamepad2.right_trigger == 0) && (gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0))
        {
            intake.powerIntake(-gamepad2.left_trigger);
            frontIntake = false;
            backIntake = false;
            resetSpinner();
        }


        powerIntake();
    }

    void powerIntake()
    {
        if(frontIntake && canIntake)
        {
            intake.powerIntake(powerIntake);

        }
        else if(backIntake && canIntake)
        {
            intake.powerIntake(-powerIntake);
        }
        else if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0 && gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0)
        {
            intake.stop();
        }

    }

    void resetSpinner()
    {
        //spinner.setUsePID(true);
    }

    void elevatorSwitch() {
        switch (elevatorMovement) {
            case SPIN:
                resetElevator();
                elevator.update();
                spinner.update();
                spinner.setSlowMove(false);
                if(withoutPID())
                {
                    handMoving();
                }
                gamepad.setCanTwist(true);

                if (gamepad1.right_bumper)
                {
                    spinner.setUsePID(true);
                    spinner.setSlowMove(true);
                    gamepad.setCanTwist(false);
                    elevator.setUsePID(true);
                    powerElevator = powerSlowElevator;
                    elevator.setPower(powerElevator);
                    frontIntake = false;
                    backIntake = false;
                    canIntake = false;
                    //intakeFixingThread.setOverride(true);
                    if(!withoutPID())
                    {
                        dip.holdFreight();
                    }

                    switch (elevatorLevel)
                    {
                        case 0:
                            elevatorMovement = ElevatorMovement.SHARED;
                            break;
                        case 1:
                            elevatorMovement = ElevatorMovement.LEVEL1;
                            break;
                        case 2:
                            elevatorMovement = ElevatorMovement.LEVEL2;
                            break;
                        case 3:
                            elevatorMovement = ElevatorMovement.LEVEL3;
                            break;
                    }

                    resetElevator.reset();
                }
                break;
            case LEVEL1:
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL1);
                if(!withoutPID() && resetElevator.seconds() > .5)
                {
                    hand.level1();
                    elevatorMovement = ElevatorMovement.DIP;
                }
                break;
            case LEVEL2:
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL2);
                if(!withoutPID() && resetElevator.seconds() > .4)
                {
                    hand.level2();
                    elevatorMovement = ElevatorMovement.DIP;
                }
                break;
            case LEVEL3:
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3);
                if(!withoutPID() && resetElevator.seconds() > .3)
                {
                    hand.level3();
                    elevatorMovement = ElevatorMovement.DIP;
                }
                break;
            case SHARED:
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.SHARED_HUB);
                if(!withoutPID() && resetElevator.seconds() > .5)
                {
                    hand.shared();
                    elevatorMovement = ElevatorMovement.DIP;
                }
                break;
            case DIP:
                elevator.update();
                spinner.update();
                spinner.setUsePID(false);
                if(withoutPID())
                {
                    handMoving();
                }
                if(gamepad1.right_stick_button)
                {
                    gamepad.setCanTwist(false);
                    spinner.setSlowMove(true);
                }
                else
                {
                    gamepad.setCanTwist(true);
                    spinner.setSlowMove(false);
                }

                if(gamepad2.right_trigger == 1 && gamepad2.left_trigger == 1)
                {
                    dip.releaseFreight();
                }

                if(gamepad2.right_stick_button && gamepad2.left_stick_button)
                {
                    switch (spinner.spinnerState)
                    {
                        case RIGHT:
                            spinner.setRIGHT_ANGLE(spinner.getPosition() + spinner.getZERO_ANGLE());
                            break;
                        case LEFT:
                            spinner.setLEFT_ANGLE(spinner.getPosition() + spinner.getZERO_ANGLE());
                            break;
                        case SHARED_BLUE:
                            spinner.setLEFT_ANGLE_SHARED(spinner.getPosition() + spinner.getZERO_ANGLE());
                            break;
                        case SHARED_RED:
                            spinner.setRIGHT_ANGLE_SHARED(spinner.getPosition() + spinner.getZERO_ANGLE());
                            break;
                        case ZERO_RED:
                            spinner.setZERO_ANGLE_RED(spinner.getPosition() + spinner.getZERO_ANGLE());
                            break;
                        case ZERO_BLUE:
                            spinner.setZERO_ANGLE_BLUE(spinner.getPosition() + spinner.getZERO_ANGLE());
                            break;
                        case ZERO_DO_NOT_USE:
                            spinner.setZERO_ANGLE(spinner.getPosition() + spinner.getZERO_ANGLE());
                            break;
                    }

                    switch (elevator.elevatorLevel)
                    {
                        case ZERO:
                            elevator.setZeroHeight(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                            break;
                        case HUB_LEVEL1:
                            elevator.setHubLevel1(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                            break;
                        case HUB_LEVEL2:
                            elevator.setHubLevel2(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                            break;
                        case HUB_LEVEL3:
                            elevator.setHubLevel3(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                            break;
                        case SHARED_HUB:
                            elevator.setSharedHub(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                            break;
                    }
                }

                switch (elevatorLevel)
                {
                    case 0:
                }

                if(gamepad1.left_bumper)
                {

                    switch (spinner.spinnerState)
                    {
                        case RIGHT:
                            spinner.setRIGHT_ANGLE(spinner.getPosition() + spinner.getZERO_ANGLE());
                            break;
                        case LEFT:
                            spinner.setLEFT_ANGLE(spinner.getPosition() + spinner.getZERO_ANGLE());
                            break;
                        case SHARED_BLUE:
                            spinner.setLEFT_ANGLE_SHARED(spinner.getPosition() + spinner.getZERO_ANGLE());
                            break;
                        case SHARED_RED:
                            spinner.setRIGHT_ANGLE_SHARED(spinner.getPosition() + spinner.getZERO_ANGLE());
                            break;
                        case ZERO_RED:
                            spinner.setZERO_ANGLE_RED(spinner.getPosition() + spinner.getZERO_ANGLE());
                            break;
                        case ZERO_BLUE:
                            spinner.setZERO_ANGLE_BLUE(spinner.getPosition() + spinner.getZERO_ANGLE());
                            break;
                        case ZERO_DO_NOT_USE:
                            spinner.setZERO_ANGLE(spinner.getPosition() + spinner.getZERO_ANGLE());
                            break;
                    }

                    switch (elevator.elevatorLevel)
                    {
                        case ZERO:
                            elevator.setZeroHeight(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                            break;
                        case HUB_LEVEL1:
                            elevator.setHubLevel1(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                            break;
                        case HUB_LEVEL2:
                            elevator.setHubLevel2(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                            break;
                        case HUB_LEVEL3:
                            elevator.setHubLevel3(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                            break;
                        case SHARED_HUB:
                            elevator.setSharedHub(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                            break;
                    }

                    if(!withoutPID())
                    {
                        dip.releaseFreight();
                    }

                    switch (elevatorLevel)
                    {
                        case 0:
                            spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.SHARED_RED);
                            break;
                        case 1:
                        case 2:
                        case 3:
                            spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
                            break;
                    }

                    gamepad.setCanTwist(true);
                    resetElevator.reset();

                    canIntake = true;
                    frontIntake = true;

                    if(spinner.getSpinnerState() == ElevatorSpinnerLibraryPID.SpinnerState.SHARED_RED)
                    {
                        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.SHARED_HUB);
                        resetElevator.reset();
                    }

                    elevatorMovement = ElevatorMovement.SPIN;
                }
                break;
            default:
                elevatorMovement = ElevatorMovement.SPIN;
                break;
        }
    }

    void resetElevator()
    {

        if(!withoutPID())
        {
            dip.releaseFreight();
        }

        elevator.setPower(powerSlowElevator);
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);

        switch (elevatorLevel)
        {
            case 0:
                hand.intake();
                if(resetElevator.seconds() > .67 && pidToggles())
                {
                    elevator.setUsePID(true);
//                    spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.SHARED_RED);
                }
                else
                {
                    elevator.setUsePID(false);
                }
                break;
            case 1:
                hand.intake();
                if(resetElevator.seconds() > 1.3 && pidToggles())
                {
                    elevator.setUsePID(true);
//                    spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
                }
                else
                {
                    elevator.setUsePID(false);
                }
                break;
            case 2:
                hand.intake();
                if(resetElevator.seconds() > .8 && pidToggles())
                {
                    elevator.setUsePID(true);
//                    spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
                }
                else
                {
                    elevator.setUsePID(false);
                }
                break;
            case 3:
                if(resetElevator.seconds() > .6 && pidToggles())
                {
                    if(!withoutPID())
                    {
                        hand.intake();
                    }
                    elevator.setUsePID(true);
//                    spinner.setSpinnerState(ElevatorSpinnerLibraryPID.SpinnerState.RIGHT);
                }
                else
                {
                    elevator.setUsePID(false);
                }
                break;
        }

        canIntake = true;
    }

    void resetElevatorMidMoving()
    {
        if(gamepad1.left_bumper && elevatorMovement != ElevatorMovement.SPIN)
        {
            elevatorMovement = ElevatorMovement.SPIN;
            canIntake = true;
            frontIntake = true;
        }

    }

    boolean withoutPID()
    {
        if(elevator.getUsePID() == true || spinner.getUsePID() == true && elevator.getElevatorLevel() != ElevatorFirstPID.ElevatorLevel.ZERO)
        {
            switch (hand.getHandPos())
            {
                case SHARED_HUB:
                    hand.shared();
                    break;
                case INTAKE:
                    break;
                case ONE_HUB:
                    hand.level1();
                    break;
                case TWO_HUB:
                    hand.level2();
                    break;
                case THREE_HUB:
                    hand.level3();
                    break;
            }
            return false;
        }
        else
        {
            return true;
        }
    }


}