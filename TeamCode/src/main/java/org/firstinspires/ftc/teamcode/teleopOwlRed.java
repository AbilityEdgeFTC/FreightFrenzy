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
import org.firstinspires.ftc.teamcode.robot.subsystems.Spinner;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.carousel;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.gamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;


/**
FOR ANY QUESTIONS PLEASE ASK ME, ELIOR YOUSEFI FROM ABILITY EDGE #18273. THIS CODE IS VERY COMPLEX AND YET, IM A LAZY
TO ADD COMMENTS. SO FOR NOW, EVERY PART THAT YOU DONT GET, PLEASE!!!!! ASK ME.
MY PHONE NUMBER IS: 050-474-7845
 */
@Config
@TeleOp(name = "TeleOp RED Alliance", group = "red")
public class teleopOwlRed extends LinearOpMode {

    gamepad gamepad;
    Spinner spinner;
    ElevatorFirstPID elevator;
    carousel carousel;
    intake intake;
    hand hand;
    dip dip;
    private boolean frontIntake = false, backIntake = false;
    cGamepad cGamepad1, cGamepad2;
    public static double powerIntake = 1, powerSlowElevator = .6, powerElevator = 1;
    boolean canIntake = true;
    double positionDip = 0;
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

    int elevatorLevel = 3;
    public static ElevatorMovement elevatorMovement = ElevatorMovement.SPIN;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepad.setRedAlliance(true);
        gamepad = new gamepad(hardwareMap, gamepad1, gamepad2, telemetry); // teleop(gamepad) class functions
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // dashboard telemetry
        spinner = new Spinner(hardwareMap, gamepad1, gamepad2);
        elevator = new ElevatorFirstPID(hardwareMap, gamepad2);
        carousel = new carousel(hardwareMap);
        intake = new intake(hardwareMap);
        hand = new hand(hardwareMap);
        dip = new dip(hardwareMap);
        cGamepad1 = new cGamepad(gamepad1);
        cGamepad2 = new cGamepad(gamepad2);
        resetElevator = new ElapsedTime();

        // wait till after init
        waitForStart();

        while (opModeIsActive()) {
            cGamepad1.update();
            cGamepad2.update();
            gamepad.update();
            elevator.update();
            spinner.updateGamepad();
            toggleCarouselGP2();
            toggleIntakesGP1GP2();
            elevatorSwitch();
            resetElevatorMidMoving();
            turnOnOfPidByUserAndReturnIfItWasChanged();
            manualServoMoving();
            switchElevatorLevelsGP2();

            telemetry.update();

        }

        gamepad.saveIMUHeading();
    }

    /**
     * Function changes the state of carousel(on,off) by the dpad left and right.
     */
    void toggleCarouselGP2()
    {
        if(gamepad2.dpad_right)
        {
            carousel.spin(false, true);
        }
        else if(gamepad2.dpad_left)
        {
            carousel.spin(true, false);
        }
        else
        {
            carousel.stop();
        }
    }

    /**
     * Function gets a elevator level, and check if the current position of the elevator is greater than it's offset so that
     * means that the elevator is open, and we want to change it's current level.
     */
    void switchElevatorLevelIfIsOpen(ElevatorFirstPID.ElevatorLevel level)
    {
        if(elevator.getTargetPosition() > elevator.getOffset())
        {
            elevator.setElevatorLevel(level);
        }
    }

    /**
     * Function changes the target elevator level by user input, and if the elvator is open, it will change it's
     * current level.
     */
    void switchElevatorLevelsGP2()
    {
        // each button on game pad sets the target level of the elevator to a different level
        if(gamepad2.a)
        {
            elevatorLevel = 1;
            switchElevatorLevelIfIsOpen(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL1);
        }
        else if(gamepad2.b)
        {
            elevatorLevel = 2;
            switchElevatorLevelIfIsOpen(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL2);
        }
        else if(gamepad2.y)
        {
            elevatorLevel = 3;
            switchElevatorLevelIfIsOpen(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3);
        }
        else if(gamepad2.x)
        {
            elevatorLevel = 0;
            elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
        }
    }

    /**
     * Function lets the hand servo go up and down with user input on gamepad 2 bumbers
     */
    void moveHandServoManuallyGP2()
    {
        // get current pos
        double position = hand.getPos();

        // check if the user wants to make the hand go up or down, and it can(0-1 range)
        if(cGamepad2.rightBumperOnce() && hand.getPos() >= 0.03)
        {
            // if so, go up
            position -= 0.03;
            hand.moveTo(position);
        }
        else if(cGamepad2.leftBumperOnce() && hand.getPos() <= 0.97)
        {
            // else if so, go down
            position += 0.03;
            hand.moveTo(position);
        }
    }

    /**
     * FUNCTION CURRENTLY DISABLED!!!!
     * Function lets the dip servo go up and down with user input on gamepad 2 dpads up and down
     */
    void manual1ServoDipMoving()
    {
        // get current pos
        positionDip = dip.getPos();

        // check if the user wants to make the dip go up or down, and it can(0-1 range)
        /*if(cGamepad2.dpadDownOnce() && dip.getPos() >= 0.02)
        {
            // if so, go up
            positionDip -= 0.02;
            dip.moveTo(positionDip);
        }
        else if(cGamepad2.dpadUpOnce() && dip.getPos() <= 0.98)
        {
            // if so, go down
            positionDip += 0.02;
            dip.moveTo(positionDip);
        }*/

    }

    boolean turnOnOfPidByUserAndReturnIfItWasChanged()
    {
        // honestly, i dont know what the fuck is this!
        if(gamepad2.left_stick_button || gamepad2.right_stick_y != 0 || gamepad2.right_stick_x != 0
                || gamepad2.left_stick_y != 0 || gamepad2.left_stick_x != 0 && !gamepad2.right_stick_button)
        {
            elevator.setUsePID(false);
            return false;
        }
        else if((gamepad2.left_trigger == 1 && gamepad2.right_trigger == 1) || gamepad1.left_bumper &&
                !gamepad2.left_stick_button && elevatorMovement != ElevatorMovement.SPIN)
        {
            elevator.setUsePID(true);
        }

        return true;
    }

    /**
     * Function toggles the intake forward/backward/stop with the gamepads 1 and 2
     */
    void toggleIntakesGP1GP2()
    {
        /**
        theres alot of if's because we want to make sure there isn't any collusion between the intake when 2 drivers are trying to power
        it by accident...
         but we just check if the gamepads want to power the intake, and which way.
         */

        if ((gamepad1.right_trigger != 0) && canIntake && (gamepad2.right_trigger == 0) && (gamepad2.left_trigger == 0) && (gamepad1.left_trigger == 0))
        {
            intake.powerIntake(gamepad1.right_trigger);
            frontIntake = false;
            backIntake = false;
        }
        else if ((gamepad1.left_trigger != 0) && canIntake && (gamepad2.right_trigger == 0) && (gamepad1.right_trigger == 0) && (gamepad2.left_trigger == 0))
        {
            intake.powerIntake(-gamepad1.left_trigger);
            frontIntake = false;
            backIntake = false;
        }
        else if ((gamepad2.right_trigger != 0) && canIntake && (gamepad2.left_trigger == 0) && (gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0))
        {
            intake.powerIntake(gamepad2.right_trigger);
            frontIntake = false;
            backIntake = false;
        }
        else if ((gamepad2.left_trigger != 0) && canIntake && (gamepad2.right_trigger == 0) && (gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0))
        {
            intake.powerIntake(-gamepad2.left_trigger);
            frontIntake = false;
            backIntake = false;
        }


        toggleIntake();
    }

    /**
     * Function toggles intake by the booleans with no need of holding buttons.
     */
    void toggleIntake()
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

    /**
     * Functions check that the pid is on, and we move the dip box to a closing position.
     */
    void closeDipBox()
    {
        if(!withoutPID())
        {
            dip.holdFreight();
        }
    }

    /**
     * Function checks that the pid is off, and turns off manual hand and dip servo control.
     */
    void manualServoMoving()
    {
        if(withoutPID())
        {
            moveHandServoManuallyGP2();
            manual1ServoDipMoving();
        }
    }

    void elevatorSwitch() {
        switch (elevatorMovement) {
            case SPIN:
                resetElevator();
                spinner.setSlowMove(false);
                gamepad.setCanTwist(true);

                if (gamepad1.right_bumper)
                {
                    elevator.setUsePID(true);
                    spinner.setSlowMove(true);
                    gamepad.setCanTwist(false);

                    powerElevator = powerSlowElevator;
                    elevator.setPower(powerElevator);
                    frontIntake = false;
                    backIntake = false;
                    canIntake = false;

                    closeDipBox();

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
                dip.holdFreight();
                if(!withoutPID() && resetElevator.seconds() > .5)
                {
                    hand.level1();
                    elevatorMovement = ElevatorMovement.DIP;
                }
                break;
            case LEVEL2:
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL2);
                dip.holdFreight();
                if(!withoutPID() && resetElevator.seconds() > .4)
                {
                    hand.level2();
                    elevatorMovement = ElevatorMovement.DIP;
                }
                break;
            case LEVEL3:
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3);
                dip.holdFreight();
                if(!withoutPID() && resetElevator.seconds() > .1)
                {
                    hand.level3();
                    elevatorMovement = ElevatorMovement.DIP;
                }
                break;
            case SHARED:
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.SHARED_HUB);
                dip.holdFreight();
                if(!withoutPID() && resetElevator.seconds() > .5)
                {
                    hand.shared();
                    elevatorMovement = ElevatorMovement.DIP;
                }
                break;
            case DIP:
                elevator.update();
                manualServoMoving();
                if(gamepad1.right_stick_button)
                {
                    gamepad.setCanTwist(true);
                    spinner.setSlowMove(false);
                }
                else
                {
                    gamepad.setCanTwist(false);
                    spinner.setSlowMove(true);
                }

                if(gamepad1.left_bumper || gamepad2.right_trigger == 1 && gamepad2.left_trigger == 1)
                {
                    spinner.setMaxPower(.4);
                    elevator.setMaxPower(.85);
                    dip.releaseFreight();

                    switch (elevatorLevel) {
                        case 0:
                            elevator.setSharedHub(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                            break;
                        case 1:
                            elevator.setHubLevel1(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                            break;
                        case 2:
                            elevator.setHubLevel2(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                            break;
                        case 3:
                            elevator.setHubLevel3(elevator.encoderTicksToInches(elevator.getPosition()) + elevator.getZeroHeight());
                            break;
                    }

                    gamepad.setCanTwist(true);
                    resetElevator.reset();

                    canIntake = true;
                    frontIntake = true;

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
        elevator.update();

        if(!withoutPID())
        {
            dip.releaseFreight();
        }

        elevator.setPower(powerSlowElevator);
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);

        switch (elevatorLevel)
        {
            case 0:
                //if(!withoutPID())
                //{
                    hand.intake();
                //}
                if(resetElevator.seconds() > .67 && turnOnOfPidByUserAndReturnIfItWasChanged())
                {
                    elevator.setUsePID(true);
                }
                else
                {
                    elevator.setUsePID(false);
                }
                break;
            case 1:
                //if(!withoutPID())
                //{
                    hand.intake();
                //}
                if(resetElevator.seconds() > 1.3 && turnOnOfPidByUserAndReturnIfItWasChanged())
                {
                    elevator.setUsePID(true);
                }
                else
                {
                    elevator.setUsePID(false);
                }
                break;
            case 2:
                //if(!withoutPID())
                //{
                    hand.intake();
                //}
                if(resetElevator.seconds() > 1 && turnOnOfPidByUserAndReturnIfItWasChanged())
                {
                    elevator.setUsePID(true);
                }
                else
                {
                    elevator.setUsePID(false);
                }
                break;
            case 3:
                if(resetElevator.seconds() > .6 && turnOnOfPidByUserAndReturnIfItWasChanged())
                {
                    //if(!withoutPID())
                    //{
                        hand.intake();
                    //}
                    elevator.setUsePID(true);
                }
                else
                {
                    elevator.setUsePID(false);
                }
                break;
        }

        elevator.update();

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
        if(elevator.getUsePID() == true && elevator.getElevatorLevel() != ElevatorFirstPID.ElevatorLevel.ZERO)
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