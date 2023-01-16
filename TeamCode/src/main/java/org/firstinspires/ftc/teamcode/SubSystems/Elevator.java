package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Elevator {

    DcMotor mE;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Elevator(HardwareMap hardwareMap)
    {
        mE = BlocksOpModeCompanion.hardwareMap.get(DcMotor.class , "mE");
        mE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void highRod(int highRodTicks , double power){

        mE.getCurrentPosition();
        telemetry.addData(" highRodTicks", mE.getCurrentPosition());

        mE.setTargetPosition(highRodTicks);
        mE.setPower(power);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void lowRod(int lowRodTicks , double power){

        mE.getCurrentPosition();
        telemetry.addData(" lowRodTicks", mE.getCurrentPosition());

        mE.setTargetPosition(lowRodTicks);
        mE.setPower(power);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void midRod(int midRodTicks , double power){

        mE.getCurrentPosition();
        telemetry.addData(" midRodTicks", mE.getCurrentPosition());

        mE.setTargetPosition(midRodTicks);
        mE.setPower(power);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void elevatorReset(int resetTicks , double power){

        mE.getCurrentPosition();
        telemetry.addData(" midRodTicks", mE.getCurrentPosition());

        mE.setTargetPosition(resetTicks);
        mE.setPower(power);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }


}
