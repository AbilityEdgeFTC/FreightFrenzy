package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static android.os.SystemClock.sleep;

//TODO:telementry for every angle by itself
public class Gyro {

    /**
     *     Hardware Map:
     *     modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
     *     gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;
     */


    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;

    Telemetry telemetry;

    LinearOpMode linearOpMode;

    OpMode opMode;

    int rawX = modernRoboticsI2cGyro.rawX();
    int rawY = modernRoboticsI2cGyro.rawY();
    int rawZ = modernRoboticsI2cGyro.rawZ();
    int heading = modernRoboticsI2cGyro.getHeading();
    int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();

    // Read dimensionalized data from the gyro. This gyro can report angular velocities
    // about all three axes. Additionally, it internally integrates the Z axis to
    // be able to report an absolute angular Z orientation.
    AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
    float zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

    // Read administrative information from the gyro
    int zAxisOffset = modernRoboticsI2cGyro.getZAxisOffset();
    int zAxisScalingCoefficient = modernRoboticsI2cGyro.getZAxisScalingCoefficient();



    public Gyro(IntegratingGyroscope gyro, ModernRoboticsI2cGyro modernRoboticsI2cGyro, LinearOpMode linearOpMode){
        this.gyro = gyro;
        this.modernRoboticsI2cGyro = modernRoboticsI2cGyro;

        this.linearOpMode = linearOpMode;
    }

    public Gyro(IntegratingGyroscope gyro, ModernRoboticsI2cGyro modernRoboticsI2cGyro, OpMode opMode){
        this.gyro = gyro;
        this.modernRoboticsI2cGyro = modernRoboticsI2cGyro;

        this.opMode = opMode;
    }

    public void Calibrate(LinearOpMode linearOpMode){
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();

        // Wait until the gyro calibration is complete
        while (!linearOpMode.opModeIsActive() && modernRoboticsI2cGyro.isCalibrating())  {
            sleep(50);
        }

        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();
    }

    public void Calibrate(OpMode opMode){
        telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();

        // Wait until the gyro calibration is complete
        while (!linearOpMode.opModeIsActive() && modernRoboticsI2cGyro.isCalibrating() || modernRoboticsI2cGyro.isCalibrating())  {
            sleep(50);
        }

        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();
    }

    public int getRawX() {
        int rawX = modernRoboticsI2cGyro.rawX();

        return rawX;
    }

    public int getRawY() {
        int rawY = modernRoboticsI2cGyro.rawY();

        return rawY;
    }

    public int getRawZ() {
        int rawZ = modernRoboticsI2cGyro.rawZ();

        return rawZ;
    }

    public int getHeading() {
        int heading = modernRoboticsI2cGyro.getHeading();

        return heading;
    }

    public int getIntegratedZ() {
        int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();

        return integratedZ;
    }

    public AngularVelocity getRates() {
        AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);

        return rates;
    }

    public float getzAngle() {
        float zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        return zAngle;
    }

    public int getzAxisOffset() {
        int zAxisOffset = modernRoboticsI2cGyro.getZAxisOffset();

        return zAxisOffset;
    }

    public int getzAxisScalingCoefficient() {
        int zAxisScalingCoefficient = modernRoboticsI2cGyro.getZAxisScalingCoefficient();

        return zAxisScalingCoefficient;
    }

    public void returnAllAngles(){
        int rawX = modernRoboticsI2cGyro.rawX();
        int rawY = modernRoboticsI2cGyro.rawY();
        int rawZ = modernRoboticsI2cGyro.rawZ();
        int heading = modernRoboticsI2cGyro.getHeading();
        int integratedZ = modernRoboticsI2cGyro.getIntegratedZValue();

        // Read dimensionalized data from the gyro. This gyro can report angular velocities
        // about all three axes. Additionally, it internally integrates the Z axis to
        // be able to report an absolute angular Z orientation.
        AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        float zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        // Read administrative information from the gyro
        int zAxisOffset = modernRoboticsI2cGyro.getZAxisOffset();
        int zAxisScalingCoefficient = modernRoboticsI2cGyro.getZAxisScalingCoefficient();
    }

    public void returnX(){
        telemetry.addLine()
                .addData("dx", formatRate(rates.xRotationRate));
    }

    public void returnY(){
        telemetry.addLine()
                .addData("dy", formatRate(rates.yRotationRate));
    }

    public void returnZ(){
        telemetry.addLine()
                .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));
    }

    public void returnAngle(){
        telemetry.addData("angle", "%s deg", formatFloat(zAngle));

    }

    public void returnHeding(){
        telemetry.addData("heading", "%3d deg", heading);
    }

    public void returnIntegratedZ(){
        telemetry.addData("integrated Z", "%3d", integratedZ);

    }

    public void returnRawX(){
        telemetry.addLine()
                .addData("rawX", formatRaw(rawX));
    }

    public void returnRawY(){
        telemetry.addLine()
                .addData("rawY", formatRaw(rawY));
    }

    public void returnRawZ(){
        telemetry.addLine()
                .addData("rawZ", formatRaw(rawZ));
    }

    public void returnZOffset(){
        telemetry.addLine().addData("z offset", zAxisOffset).addData("z coeff", zAxisScalingCoefficient);

    }

    public void returnAllTelementry(){
        telemetry.addLine()
                .addData("dx", formatRate(rates.xRotationRate))
                .addData("dy", formatRate(rates.yRotationRate))
                .addData("dz", "%s deg/s", formatRate(rates.zRotationRate));
        telemetry.addData("angle", "%s deg", formatFloat(zAngle));
        telemetry.addData("heading", "%3d deg", heading);
        telemetry.addData("integrated Z", "%3d", integratedZ);
        telemetry.addLine()
                .addData("rawX", formatRaw(rawX))
                .addData("rawY", formatRaw(rawY))
                .addData("rawZ", formatRaw(rawZ));
        telemetry.addLine().addData("z offset", zAxisOffset).addData("z coeff", zAxisScalingCoefficient);
        telemetry.update();
    }

    String formatRaw(int rawValue) {
        return String.format("%d", rawValue);
    }

    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    String formatFloat(float rate) {
        return String.format("%.3f", rate);
    }



}
