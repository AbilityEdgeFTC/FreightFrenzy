package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.opmodes.Vision.HSVPipeline;
import org.firstinspires.ftc.teamcode.robot.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.robot.subsystems.Cover;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.FreightSensor;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "Auto - Right Red with old paths BUT ASYNC", group = "Autonomous Red")
public class AutoRightRedAsync extends LinearOpMode {

    double startPoseRightX = 13;
    double startPoseRightY = -72 + 17.72;
    double startPoseRightH = 90;

    public static double poseEntranceX = 13.5;
    public static double poseEntranceY = -58;
    public static double poseEntranceH = 180;
    public static double poseCollectX = 50.5;
    public static double poseCollectY = -58;
    public static double poseCollectH = 180;
    public static double poseHelpX = 7;
    public static double poseHelpY = -50;
    public static double poseHelpH = 180;

    //Cordinates for each course
    public static double cylceX2 = 60;
    public static double cycleY2 = -58;
    public static double cycleH2 = 180;

    ElevatorFirstPID elevator;
    SpinnerFirstPID spinner;
    hand hand;
    intake intake;
    dip dip;
    Cover cover;
    ElapsedTime runningFor;
    FreightSensor freightSensor;
    SampleMecanumDrive drive;

    public static double powerSlowElevator = .7, powerElevator = 1, powerElevatorFast = 1, elevatorDelay = .9;

    boolean hasFreight = false, firstTime = true;
    double offset = 0;
    int intakeNumber = 0;

    TrajectorySequence fixAngle, goToHub1, goToIntake1, goToHub2, goToIntake2, goToHub3, goToIntake3, goToHub4, goToIntake4, goToHub5, goToPark;
    Pose2d startPoseRight, poseHelp, poseEntrance, poseCollect, poseCollectCycle2, wareHouseHelp;

    enum State
    {
        FIX_ANGLE,
        PARK,
        OPEN_ELEVATOR,
        WAIT_ELEVATOR_DELAY,
        INTAKE,
        IDLE
    }

    State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        elevator = new ElevatorFirstPID(hardwareMap);
        spinner = new SpinnerFirstPID(hardwareMap);
        cover = new Cover(hardwareMap);
        intake = new intake(hardwareMap);
        hand = new hand(hardwareMap);
        dip = new dip(hardwareMap);
        freightSensor = new FreightSensor(hardwareMap);

        DriveConstants.setMaxVel(65);
        DriveConstants.setMaxAccel(65);

        TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(50),
                new RectangleMaskConstraint(47,-72,72,-47,
                        new TranslationalVelocityConstraint(10))));

        TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(40);

        startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, Math.toRadians(startPoseRightH));
        poseHelp = new Pose2d(poseHelpX, poseHelpY, Math.toRadians(poseHelpH));
        poseEntrance = new Pose2d(poseEntranceX, poseEntranceY, Math.toRadians(poseEntranceH));
        poseCollect = new Pose2d(poseCollectX, poseCollectY, Math.toRadians(poseCollectH));
        poseCollectCycle2 = new Pose2d(cylceX2 , cycleY2, Math.toRadians(cycleH2));

        MarkerCallback intakeForward =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                dip.getFreight();
                intake.intakeForward();
                elevator.updateAuto();
                spinner.updateAuto();
            }
        };

        MarkerCallback intakeBackword =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                intake.intakeBackward();
                elevator.updateAuto();
                spinner.updateAuto();
            }
        };


        MarkerCallback intakeStop =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                intake.stop();
            }
        };

        drive.setPoseEstimate(startPoseRight);

        dip.getFreight();
        cover.closeCover();

        fixAngle = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(poseHelp)
                .build();

        goToHub1 = drive.trajectorySequenceBuilder(fixAngle.end())
                .lineToLinearHeading(poseEntrance)
                .build();

        goToIntake1 = new TrajectorySequenceBuilder(goToHub1.end(), velConstraint, accelConstraint,
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(new Pose2d(poseCollect.getX()+2, poseCollect.getY(), poseCollect.getHeading()),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
            SampleMecanumDrive.getAccelerationConstraint(40))
                .build();

        goToHub2 = drive.trajectorySequenceBuilder(goToIntake1.end())
                .addTemporalMarker(intakeBackword)
                .lineToLinearHeading(poseEntrance)
                .build();

        goToIntake2 = new TrajectorySequenceBuilder(goToHub2.end(), velConstraint, accelConstraint,
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToLinearHeading(new Pose2d(poseCollectCycle2.getX(), poseCollectCycle2.getY(), poseCollectCycle2.getHeading())
                        ,SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToSplineHeading(new Pose2d(poseCollect.getX()+6, poseCollect.getY(), poseCollect.getHeading()),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .build();

        goToHub3 = drive.trajectorySequenceBuilder(goToIntake2.end())
                .addTemporalMarker(intakeBackword)
                .lineToLinearHeading(poseEntrance)
                .build();

        goToIntake3 = new TrajectorySequenceBuilder(goToHub3.end(), velConstraint, accelConstraint,
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .addTemporalMarker(intakeForward)
                .lineToLinearHeading(new Pose2d(poseCollectCycle2.getX(), poseCollectCycle2.getY(), poseCollectCycle2.getHeading())
                        ,SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToSplineHeading(new Pose2d(poseCollect.getX()+6, poseCollect.getY(), poseCollect.getHeading()),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .build();

        goToHub4 = drive.trajectorySequenceBuilder(goToIntake3.end())
                .addTemporalMarker(intakeBackword)
                .lineToLinearHeading(poseEntrance)
                .build();

        goToIntake4 = new TrajectorySequenceBuilder(goToHub4.end(), velConstraint, accelConstraint,
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(new Pose2d(poseCollect.getX(), poseCollect.getY(), poseCollect.getHeading()))
                .lineToSplineHeading(new Pose2d(poseCollect.getX()+6, poseCollect.getY()+8, poseCollect.getHeading()),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .lineToSplineHeading(new Pose2d(poseCollect.getX()+6, poseCollect.getY(), poseCollect.getHeading()),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL,
                                DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .forward(4)
                .build();

        goToHub5 = drive.trajectorySequenceBuilder(goToIntake4.end())
                .addTemporalMarker(intakeBackword)
                .lineToLinearHeading(poseEntrance)
                .build();

        goToPark = drive.trajectorySequenceBuilder(goToHub5.end())
                .addTemporalMarker(intakeStop)
                .lineToSplineHeading(new Pose2d(poseCollect.getX()+5, poseCollect.getY(), poseCollect.getHeading()),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .build();

        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.ZERO_RED);

        waitForStart();

        if (isStopRequested()) return;

        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
        spinner.updateAuto();
        elevator.updateAuto();

        runningFor = new ElapsedTime();

        currentState = State.FIX_ANGLE;
        drive.followTrajectorySequenceAsync(fixAngle);

        while (opModeIsActive() && !isStopRequested()) {
            switchTrajs();

            hasFreight = freightSensor.hasFreight();

            // We update drive continuously in the background, regardless of state
            drive.update();

            elevator.updateAuto();
            spinner.updateAuto();
        }
    }

    void switchTrajs()
    {
        switch (currentState) {
            case FIX_ANGLE:
                changeState(State.OPEN_ELEVATOR, goToHub1);
                break;
            case OPEN_ELEVATOR:
                if(!drive.isBusy())
                {
                    intake.stop();
                    openElevator();
                    currentState = State.WAIT_ELEVATOR_DELAY;
                }
                break;
            case WAIT_ELEVATOR_DELAY:
                if(firstTime)
                {
                    offset = runningFor.seconds();
                    firstTime = false;
                }

                if((runningFor.seconds() - offset) >= elevatorDelay)
                {
                    firstTime = true;

                    closeElevator();

                    changeStateIntake();
                }
                break;
            case INTAKE:
                if(hasFreight)
                {
                    drive.breakFollowing();
                    intake.intakeBackward();

                    changeStateElevator();
                }
                break;
            case PARK:
                changeState(State.IDLE);
            case IDLE:
                requestOpModeStop();
                break;
        }
    }

    void changeStateElevator()
    {
        if (!drive.isBusy())
        {
            currentState = State.INTAKE;

            switch (intakeNumber)
            {
                case 1:
                    drive.followTrajectorySequenceAsync(goToHub2);
                    break;
                case 2:
                    drive.followTrajectorySequenceAsync(goToHub3);
                    break;
                case 3:
                    drive.followTrajectorySequenceAsync(goToHub4);
                    break;
                case 4:
                    drive.followTrajectorySequenceAsync(goToHub5);
                    break;
            }
        }
    }

    void changeStateIntake()
    {
        if (!drive.isBusy())
        {
            intakeNumber++;
            currentState = State.INTAKE;

            switch (intakeNumber)
            {
                case 1:
                    drive.followTrajectorySequenceAsync(goToIntake1);
                    break;
                case 2:
                    drive.followTrajectorySequenceAsync(goToIntake2);
                    break;
                case 3:
                    drive.followTrajectorySequenceAsync(goToIntake3);
                    break;
                case 4:
                    drive.followTrajectorySequenceAsync(goToIntake4);
                    break;
                case 5:
                    drive.followTrajectorySequenceAsync(goToPark);
                    currentState = State.PARK;
                    break;
            }
        }
    }

    void changeState(State newState, TrajectorySequence traj)
    {
        if (!drive.isBusy()) {
            currentState = newState;
            drive.followTrajectorySequenceAsync(traj);
        }
    }

    void changeState(State newState)
    {
        if (!drive.isBusy()) {
            currentState = newState;
        }
    }

    void openElevator()
    {
        cover.openCover();

        powerElevator = powerElevatorFast;
        elevator.setPower(powerElevator);

        elevator.updateAuto();
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3);
        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
        spinner.updateAuto();


        dip.holdFreight();

        hand.level3();

        elevator.updateAuto();
        spinner.updateAuto();
    }

    void closeElevator()
    {
        dip.releaseFreight();

        powerElevator = powerSlowElevator;
        elevator.setPower(powerElevator);

        elevator.updateAuto();
        spinner.updateAuto();

        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);

        elevator.updateAuto();
        spinner.updateAuto();

        hand.intake();

        cover.closeCover();
    }


}
