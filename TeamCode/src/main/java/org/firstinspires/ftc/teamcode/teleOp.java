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
@TeleOp(name = "RED TeleOp - Driver Control", group = "DriverControl")
public class teleOp extends LinearOpMode {

    gamepad gamepad;
    Spinner spinner;
    ElevatorFirstPID elevator;
    carousel carousel;
    intake intake;
    hand hand;
    dip dip;
    boolean frontIntake = false, backIntake = false, canIntake = true;
    cGamepad cGamepad1, cGamepad2;
    public static double powerIntake = 1, powerSlowElevator = .6, powerElevator = 1;
    double positionDip = 0;
    ElapsedTime resetElevator;

    enum ElevatorMovement
    {
        CLOSED,
        LEVEL1,
        LEVEL2,
        LEVEL3,
        SHARED,
        DIP
    }

    int elevatorLevel = 3;
    public static ElevatorMovement elevatorMovement = ElevatorMovement.CLOSED;

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
                !gamepad2.left_stick_button && elevatorMovement != ElevatorMovement.CLOSED)
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
     * Function gets a position to move the dip, and if the pid is on so move to that position.
     * @param position the poisiton to move the dip
     */
    void moveAutomaticallyDip(double position)
    {
        if(!withoutPID())
        {
            dip.moveTo(position);
        }
    }

    /**
     * Function checks that the pid is off, and turns on manual hand and dip servo control.
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
            case CLOSED:
                // while elevator is closed, spinner manual moving should be regular, and the gamepad 1 should let the user
                // to spin the robot and not the spinner
                resetElevator();
                spinner.setSlowMove(false);
                gamepad.setCanTwist(true);

                // user wants to open elevator
                if (gamepad1.right_bumper)
                {
                    // turn on pid if it was off, spinner movement should be slower and the gamepad1 should twist the spinner
                    // not the robot
                    elevator.setUsePID(true);
                    spinner.setSlowMove(true);
                    gamepad.setCanTwist(false);

                    // open elevator slowly??? idk way lemme check this
                    //powerElevator = powerSlowElevator;
                    //elevator.setPower(powerElevator);

                    // turn of intake, and the option for intake
                    frontIntake = false;
                    backIntake = false;
                    canIntake = false;

                    // move to closing dip position
                    moveAutomaticallyDip(dip.getHoldingPosition());

                    // go to the next state by the elevator level
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

                    //resetElevator.reset();
                }
                break;
            case LEVEL1:
                // move elevator to the target, and add a delay for the opening of the hand
                setTargetLevelCloseDipAndWaitTillDelayForHand(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL1, .5, hand.getLevel1Hub());
                break;
            case LEVEL2:
                // move elevator to the target, and add a delay for the opening of the hand
                setTargetLevelCloseDipAndWaitTillDelayForHand(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL2, .4, hand.getLevel2Hub());
                break;
            case LEVEL3:
                // move elevator to the target, and add a delay for the opening of the hand
                setTargetLevelCloseDipAndWaitTillDelayForHand(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3, .1, hand.getLevel3Hub());
                break;
            case SHARED:
                // move elevator to the target, and add a delay for the opening of the hand
                setTargetLevelCloseDipAndWaitTillDelayForHand(ElevatorFirstPID.ElevatorLevel.SHARED_HUB, .5, hand.getLevelSharedHub());
                break;
            case DIP:
                // this will be in a loop, so update elevator and turn on manual servo moving
                elevator.update();
                manualServoMoving();

                // if gp1 right stick button is on, so we want to twist the robot and not spinner
                if(gamepad1.right_stick_button)
                {
                    gamepad.setCanTwist(true);
                    spinner.setSlowMove(false);
                }
                // else we want to twist spinner not robot
                else
                {
                    gamepad.setCanTwist(false);
                    spinner.setSlowMove(true);
                }

                // user wants to close elevator
                if(gamepad1.left_bumper || gamepad2.right_trigger == 1 && gamepad2.left_trigger == 1)
                {
                    // release freight
                    dip.releaseFreight();

                    // make spinner slower and close elevator faster
                    spinner.setMaxPower(.4);
                    elevator.setMaxPower(.85);

                    // save the current position of the elevator
                    saveCurrentHubLevel();

                    // reset eelvator timer
                    resetElevator.reset();

                    // let the intake work and turn it on forwardly
                    canIntake = true;
                    frontIntake = true;

                    // go to the closing state
                    elevatorMovement = ElevatorMovement.CLOSED;
                }
                break;
            default:
                elevatorMovement = ElevatorMovement.CLOSED;
                break;
        }
    }

    /**
     * Function sees which level is the elevator now, and saves it that way the next time we go to that level, it goes to the
     * lastly closed position.
     */
    void saveCurrentHubLevel()
    {
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
    }

    /**
     * Function seta new targel level, closes the dip and waits a delay till we move the hand
     * @param elevatorLevel new elevator level
     * @param delay the amount to wait before opening the hand
     * @param pos where should the hand move to
     */
    void setTargetLevelCloseDipAndWaitTillDelayForHand(ElevatorFirstPID.ElevatorLevel elevatorLevel, double delay, double pos)
    {
        // set new elevator level
        elevator.setElevatorLevel(elevatorLevel);

        // close dip
        dip.holdFreight();

        // wait the delay(if with pid on ofc)
        if(!withoutPID() && resetElevator.seconds() > delay)
        {
            // move hand to the pos and go on to the next state in the finite state machine.
            hand.moveTo(pos);
            elevatorMovement = ElevatorMovement.DIP;
        }
    }

    /**
     * Function resets the dip, slowly closes the elevator, let's the intake to turn on, and closes the hand with/without delay.
     */
    void resetElevator()
    {
        // open dip
        moveAutomaticallyDip(dip.getReleasingPosition());

        // close slowly elevator
        slowCloseElevator();

        // close the hand with a delay
        switch (elevatorLevel)
        {
            case 0:
                delayEleavtorCloseAndReturnHand(.63);
                break;
            case 1:
                delayEleavtorCloseAndReturnHand(1.3);
                break;
            case 2:
                delayEleavtorCloseAndReturnHand(1);
                break;
            case 3:
                delayFullRetractionOfElevator(.6);
                break;
        }

        // robot can intake, and update the elevator
        elevator.update();
        canIntake = true;
    }

    /**
     * Function closes the elevator with a slower poer
     */
    void slowCloseElevator()
    {
        elevator.setPower(powerSlowElevator);
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
    }

    /**
     * Function returns delays the retraction of the hand to intake position, and elevator closing
     * @param delay the delay till the full elevator retraction
     */
    void delayFullRetractionOfElevator(double delay)
    {
        // wait delay long
        if(resetElevator.seconds() > delay && turnOnOfPidByUserAndReturnIfItWasChanged())
        {
            // return hand
            hand.intake();

            // turn on pid
            elevator.setUsePID(true);
        }
        else
        {
            elevator.setUsePID(false);
        }
    }

    /**
     * Function returns the hand to intake position, and adds a delay till the elevator will close
     * @param delay the delay till the elevator closes
     */
    void delayEleavtorCloseAndReturnHand(double delay)
    {
        // return hand to intake pos
        hand.intake();

        // check if delay has gone by, if yes turn on pid and return elevator, else, don't
        if(resetElevator.seconds() > delay && turnOnOfPidByUserAndReturnIfItWasChanged())
        {
            elevator.setUsePID(true);
        }
        else
        {
            elevator.setUsePID(false);
        }
    }

    /**
     * Check if the user wants the reset the elevator while it's moving in the finite state machine, if so
     * reset and turn on intake
     */
    void resetElevatorMidMoving()
    {
        // does user want to reset mid moving, and the elevator isn't closed?
        if(gamepad1.left_bumper && elevatorMovement != ElevatorMovement.CLOSED)
        {
            // reset elevator logic
            elevatorMovement = ElevatorMovement.CLOSED;

            // turn on intake
            canIntake = true;
            frontIntake = true;
        }

    }

    /**
     * Function check is elevator pid is on and at level 0, if so set the hand to the correct position and return false, else return true
     */
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