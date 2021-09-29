package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class imu {

    /**
     * declaring telemetry for printing the heading, roll and pitch.
     */
    private Telemetry telemetry;

    /**
     * declaring the imu and his orientation
     */
        BNO055IMU IMU;
        Orientation angles;

        /**creating an object for the parameters
         *
         */
        BNO055IMU.Parameters Parameters=new BNO055IMU.Parameters();

        /** 2 basic constructors,one for the imu and another one for the angles
         *
         * @param Imu
         */
    public imu(BNO055IMU Imu)
    {
      this.IMU=Imu;
    }


    public imu(Orientation Angles)
    {
      this.angles=Angles;
    }

        /**constructor for the parameters,here we set it up for degrees
         *
         * @param parameters
         */
    public imu(BNO055IMU.Parameters parameters)
    {
      this.Parameters=parameters;

      parameters.angleUnit=BNO055IMU.AngleUnit.DEGREES;
      parameters.calibrationDataFile="BN055IMUCalibration.json";
    }

        /** the print of the telemtry,shows heading,roll and pitch
         *
         * @param angles
         */
    public void  toString(Orientation angles)
    {
        telemetry.addData("Heading",this.angles.firstAngle);
        telemetry.addData("Roll",this.angles.secondAngle);
        telemetry.addData("Pitch",this.angles.thirdAngle);
    }
}
