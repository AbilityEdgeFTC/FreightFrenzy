package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.robot.subsystems.Cover;
import org.firstinspires.ftc.teamcode.robot.subsystems.ElevatorFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.SpinnerFirstPID;
import org.firstinspires.ftc.teamcode.robot.subsystems.dip;
import org.firstinspires.ftc.teamcode.robot.subsystems.hand;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import java.util.Arrays;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(name = "Auto - Right Red ASYNC", group = "Autonomous Red")
public class AutoRightRedAsync extends LinearOpMode {

    public static double startPoseRightX = 13;
    public static double startPoseRightY = -54.28;
    public static double startPoseRightH = 90;

    public static double poseFixAngleX = 7;
    public static double poseFixAngleY = -50;
    public static double poseFixAngleH = 180;
    public static double poseHubX = 12.5;
    public static double poseHubY = -58;
    public static double poseHubH = 180;
    public static double poseIntakeX = 50;
    public static double poseIntakeY = -58;
    public static double poseIntakeH = 180;
    public static double poseIntakeFifteenX = 60;
    public static double poseIntakeFifteenY = -57;
    public static double poseIntakeFifteenH = 15;
    public static double poseIntakeThirtyH = 30;

    public static double powerElevator = 1;
    public static double GO_PARK_AT = 28;

    int wentIntakeXTimes = 0;
    boolean hasFreight = false;

    SampleMecanumDrive drive;

    enum State
    {
        FIX_ANGLE, // fix angle from start angle
        INTAKE, // go to intake
        PLACE_IN_HUB, // go to placing hub position
        PARK, // park position
        IDLE, // nothing
        LEAVE_EVERYTHING_AND_PARK
    }

    enum intakePathType
    {
        STRAIGHT_PATH,
        FIFTEEN_DEGREE_PATH,
        THIRTY_DEGREE_PATH
    }

    State currentState = State.IDLE;
    intakePathType intakeType = intakePathType.STRAIGHT_PATH;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        ElevatorFirstPID elevator = new ElevatorFirstPID(hardwareMap);
        SpinnerFirstPID spinner = new SpinnerFirstPID(hardwareMap);
        hand hand = new hand(hardwareMap);
        intake intake = new intake(hardwareMap);
        dip dip = new dip(hardwareMap);
        Cover cover = new Cover(hardwareMap);

        Pose2d startPoseRight = new Pose2d(startPoseRightX, startPoseRightY, startPoseRightH);
        Pose2d poseFixAngle = new Pose2d(poseFixAngleX, poseFixAngleY, Math.toRadians(poseFixAngleH));
        Pose2d poseHub = new Pose2d(poseHubX, poseHubY, Math.toRadians(poseHubH));
        Pose2d poseGoToIntake = new Pose2d(poseIntakeX, poseIntakeY, Math.toRadians(poseIntakeH));
        Pose2d poseGoToIntakeFifteen = new Pose2d(poseIntakeFifteenX, poseIntakeFifteenY, Math.toRadians(poseIntakeFifteenH));
        Pose2d poseGoToIntakeThirty = new Pose2d(poseIntakeFifteenX, poseIntakeFifteenY, Math.toRadians(poseIntakeThirtyH));

        drive.setPoseEstimate(startPoseRight);

        MarkerCallback elevetorOpen = new MarkerCallback()
        {
            @Override
            public void onMarkerReached() {
                cover.openCover();

                elevator.setPower(powerElevator);

                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.HUB_LEVEL3);
                spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);

                elevator.updateAuto();
                spinner.updateAuto();

                hand.level3();
                dip.holdFreight();
            }
        };

        MarkerCallback elevetorClose =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                dip.releaseFreight();

                elevator.setPower(powerElevator);

                hand.intake();

                spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
                elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);

                elevator.updateAuto();
                spinner.updateAuto();

                cover.closeCover();
            }
        };

        MarkerCallback intakeForward =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                dip.getFreight();
                intake.intakeForward();
            }
        };

        MarkerCallback intakeBackword =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                intake.intakeBackward();
            }
        };

        MarkerCallback intakeStop =  new MarkerCallback()
        {
            @Override
            public void onMarkerReached(){
                intake.stop();
            }
        };


        TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(80),
                new RectangleMaskConstraint(40,-72,72,-40,
                        new TranslationalVelocityConstraint(10))));

        TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(80);


        // Let's define our trajectories
        TrajectorySequence fixAngle = drive.trajectorySequenceBuilder(startPoseRight)
                .lineToLinearHeading(poseFixAngle)
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        TrajectorySequence goToHub = drive.trajectorySequenceBuilder(fixAngle.end())
                .addTemporalMarker(intakeBackword)
                .lineToSplineHeading(poseHub)
                .addTemporalMarker(elevetorOpen)
                .waitSeconds(.8)
                .addTemporalMarker(elevetorClose)
                .build();

        TrajectorySequence straightLineIntake = new TrajectorySequenceBuilder(goToHub.end(), velConstraint, accelConstraint, DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseGoToIntake)
                .build();

        TrajectorySequence fifteenDegreeIntake = new TrajectorySequenceBuilder(goToHub.end(), velConstraint, accelConstraint, DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseGoToIntake)
                .splineTo(new Vector2d(poseGoToIntakeFifteen.getX(), poseGoToIntakeFifteen.getY()), poseGoToIntakeFifteen.getHeading())
                .build();

        TrajectorySequence thirtyDegreeIntake = new TrajectorySequenceBuilder(goToHub.end(), velConstraint, accelConstraint, DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL)
                .addTemporalMarker(intakeForward)
                .lineToSplineHeading(poseGoToIntake)
                .splineTo(new Vector2d(poseGoToIntakeThirty.getX(), poseGoToIntakeThirty.getY()), poseGoToIntakeThirty.getHeading())
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(goToHub.end())
                .addTemporalMarker(intakeStop)
                .lineToSplineHeading(poseGoToIntake)
                .build();

        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.ZERO_RED);

        waitForStart();

        spinner.setSpinnerState(SpinnerFirstPID.SpinnerState.RIGHT);
        elevator.setElevatorLevel(ElevatorFirstPID.ElevatorLevel.ZERO);
        spinner.updateAuto();
        elevator.updateAuto();

        ElapsedTime runningFor = new ElapsedTime();

        if (isStopRequested()) return;

        currentState = State.FIX_ANGLE;
        drive.followTrajectorySequenceAsync(fixAngle);

        while (opModeIsActive() && !isStopRequested()) {
            switch (currentState) {
                case FIX_ANGLE:
                    changeState(State.PLACE_IN_HUB, goToHub);
                    break;
                case PLACE_IN_HUB:
                    switch (intakeType)
                    {
                        case STRAIGHT_PATH:
                            changeStateIntake(State.INTAKE, straightLineIntake);
                            break;
                        case FIFTEEN_DEGREE_PATH:
                            changeStateIntake(State.INTAKE, fifteenDegreeIntake);
                            break;
                        case THIRTY_DEGREE_PATH:
                            changeStateIntake(State.INTAKE, thirtyDegreeIntake);
                            break;
                    }
                    break;
                case INTAKE:
                    if(hasFreight)
                    {
                        drive.breakFollowing();
                    }

                    if (!drive.isBusy()) {
                        currentState = State.PLACE_IN_HUB;
                        drive.followTrajectorySequenceAsync(goToHub);
                    }
                    break;
                case PARK:
                    changeState(State.IDLE, park);
                case IDLE:
                    break;
                case LEAVE_EVERYTHING_AND_PARK:
                    if (drive.isBusy())
                    {
                        drive.breakFollowing();
                        Pose2d currentPose = drive.getPoseEstimate();
                        TrajectorySequence parkNow = drive.trajectorySequenceBuilder(currentPose)
                                .addTemporalMarker(elevetorClose)
                                .lineToSplineHeading(poseGoToIntake)
                                .addTemporalMarker(intakeStop)
                                .build();
                        drive.followTrajectorySequenceAsync(parkNow);
                    }
                    else
                    {
                        changeState(State.IDLE, park);
                    }
            }

            if(runningFor.seconds() >= GO_PARK_AT)
            {
                currentState = State.LEAVE_EVERYTHING_AND_PARK;
            }

            // We update drive continuously in the background, regardless of state
            drive.update();

            elevator.updateAuto();
            spinner.updateAuto();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;
        }
    }

    void changeState(State newState, TrajectorySequence traj)
    {
        if (!drive.isBusy()) {
            currentState = newState;
            drive.followTrajectorySequenceAsync(traj);
        }
    }

    void changeStateIntake(State newState, TrajectorySequence traj)
    {
        if (!drive.isBusy()) {
            currentState = newState;
            drive.followTrajectorySequenceAsync(traj);

            wentIntakeXTimes++;

            switch (wentIntakeXTimes)
            {
                case 1:
                    intakeType = intakePathType.STRAIGHT_PATH;
                    break;
                case 2:
                    intakeType = intakePathType.FIFTEEN_DEGREE_PATH;
                    break;
                case 3:
                    intakeType = intakePathType.THIRTY_DEGREE_PATH;
                    wentIntakeXTimes = 0;
                    break;
            }
        }
    }

}
