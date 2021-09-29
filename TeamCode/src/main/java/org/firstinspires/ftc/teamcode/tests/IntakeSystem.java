package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

public class IntakeSystem {
    public DcMotor Intake;
    public double pwr;
    public HardwareMap hardwareMap;
    public String hardwareName;

    public IntakeSystem(HardwareMap hardwareMap,DcMotor intake, double pwr, String hardwareName) {
       this.Intake = intake;
        this.pwr = pwr;
        this.hardwareMap = hardwareMap;
        this.hardwareName = hardwareName;

        Intake = hardwareMap.get(DcMotor.class, hardwareName);

    }

    public void IntakeF() {
        Intake.setPower(pwr);

    }
    public void IntakeB() {
        Intake.setPower(-pwr);
    }
    public void IntakeStop() {
        Intake.setPower(0);
    }
}
