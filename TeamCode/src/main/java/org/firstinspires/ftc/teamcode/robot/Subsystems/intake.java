/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode.robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class intake {

    DcMotor mI;
    double power;
    Telemetry telemetry;

    /**
     * constructor for intake
     * @param mI the intake motor
     */
    public intake(DcMotor mI) {
        this.mI = mI;
    }

    /**
     * constructor for intake
     * @param mI the intake motor
     * @param power the power to give the motor
     */
    public intake(DcMotor mI, double power) {
        this.power = power;
        this.mI = mI;
    }

    /**
     * constructor for intake
     * @param mI the intake motor
     * @param power the power to give the motor
     * @param telemetry the telemetry object from the opmode
     */
    public intake(DcMotor mI, double power, Telemetry telemetry) {
        this.power = power;
        this.mI = mI;
        this.telemetry = telemetry;
    }

    /**
     * set power to mI
     * setting the power given from the user to the mI
     */
    public void powerIntake(double newPower){
        mI.setPower(newPower);
    }

    /**
     * set power to mI
     * setting the power given from the constructor to the mI
     */
    public void intakeForward(){
        mI.setPower(power);
    }

    /**
     * set -power to mI
     * setting the power in a negative value given from the constructor to the mI
     */
    public void intakeBackward(){
        mI.setPower(-power);
    }

    /**
     * set power 0 to mI
     * stopping the mI motor
     */
    public void stop(){
        mI.setPower(0);
    }

    /**
     * displaying intake motor power
     */
    public void displayTelemetry(){
        telemetry.addData("Intake motor power at", power);
        telemetry.update();
    }

}