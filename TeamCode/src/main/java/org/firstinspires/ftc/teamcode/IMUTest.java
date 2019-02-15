package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import java.util.Locale;


/**
 * Created by christianluu on 02/15/18.
 */

@TeleOp(name = "IMUTest")
//@Disabled

public class IMUTest extends LinearOpMode {

    BNO055IMU internal_imu;
    Orientation rawAngles;
    Orientation lastAngles;

    float rawHeading;
    float lastHeading;
    float globalHeading;


    @Override
    public void runOpMode() {
        telemetry.addData(">", "WAIT FOR IMU TO CALIBRATE...");
        telemetry.update();

        //SETUP for Internal IMU data logging - do not touch
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //INIT HARDWARE
        internal_imu = hardwareMap.get(BNO055IMU.class, "internal_imu");
        internal_imu.initialize(parameters);


        // Wait for the game to begin
        telemetry.addData(">", "IMU Calibrated: Press Play to start");
        resetGlobalHeading();

        telemetry.update();
        /*
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = internal_imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = internal_imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return internal_imu.getSystemStatus().toShortString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        //heading = Float.parseFloat(formatAngle(angles.angleUnit, angles.firstAngle));
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                });
                */
        waitForStart();

        while (opModeIsActive()) {
            rawAngles = internal_imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //update IMU
            rawHeading = Float.parseFloat(formatAngle(rawAngles.angleUnit, rawAngles.firstAngle));
            telemetry.addData("Heading Raw: ", rawHeading);
            telemetry.addData("Heading Global: ", getGlobalHeading());
            telemetry.update();
            //EXTRAPOLATION OF ANGLES

        }
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void resetGlobalHeading() {
        lastAngles = internal_imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalHeading = 0;
    }

    public float getGlobalHeading() {
        rawAngles = internal_imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //pull data for rawAngle
        lastHeading = Float.parseFloat(formatAngle(lastAngles.angleUnit, lastAngles.firstAngle));
        rawHeading = Float.parseFloat(formatAngle(rawAngles.angleUnit, rawAngles.firstAngle));
        //NEED TO RESET ANGLE EVERY 360!!!
        //positive to left, negative to right
        float deltaHeading = rawHeading - lastHeading;

        if (deltaHeading < -180)
            deltaHeading += 360;
        else if (deltaHeading > 180)
            deltaHeading -= 360;

        globalHeading += deltaHeading;

        lastAngles = rawAngles;

        return globalHeading;
    }
}



