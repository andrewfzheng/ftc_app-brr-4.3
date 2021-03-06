package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@Autonomous(name = "Test_IMUDepotAuto", group = "DogeCV")


public class Test_IMUDepotAuto extends LinearOpMode {

    // Hardware declaration
    private DcMotor upMotor;
    private DcMotor downMotor;
    private DcMotor flDrive;
    private DcMotor frDrive;
    private DcMotor rlDrive;
    private DcMotor rrDrive;
    private DcMotor intakeSpinMotor;
    private Servo markerArm;
    private Servo dispServo;
    private Servo intakeFlipServo;
    private Servo dispExtensionServo;

    // Variables for hardware
    double intakeFlipServoUp = .92;
    double intakeFlipServoLowMid = 0.61;
    double intakeFlipServoDown = 0.11;
    double intakeFlipServoTrueMid = 0.35;

    double dispServoUp = 0.094;
    double dispServoDown = 0.80;

    double dispExtensionServoIn = 0.67;
    double dispExtensionServoOut = 0.11;

    double markerArmUp = 0.6;
    double markerArmDown = 0.07;

    int LiftPower = 1;

    double pos = 0;

    // Detector object
    private GoldAlignDetector detector;
    private ElapsedTime runtime = new ElapsedTime();
    String mineralPos = "none";

    // Imu setup
    BNO055IMU internal_imu;
    Orientation rawAngles;
    Orientation lastAngles;

    // Imu variables
    float rawHeading;
    float lastHeading;
    float globalHeading;


    @Override
    public void runOpMode() throws InterruptedException {

        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment
        detector.enable();

        flDrive = hardwareMap.get(DcMotor.class, "fl_drive");
        frDrive = hardwareMap.get(DcMotor.class, "fr_drive");
        rlDrive = hardwareMap.get(DcMotor.class, "rl_drive");
        rrDrive = hardwareMap.get(DcMotor.class, "rr_drive");
        upMotor = hardwareMap.get(DcMotor.class, "up_motor");
        downMotor = hardwareMap.get(DcMotor.class, "down_motor");
        markerArm = hardwareMap.get(Servo.class, "marker_servo");
        dispServo = hardwareMap.get(Servo.class, "disp_servo");
        dispExtensionServo = hardwareMap.get(Servo.class, "disp_extend_servo");
        intakeFlipServo = hardwareMap.get(Servo.class, "intake_flip_servo");
        intakeSpinMotor = hardwareMap.get(DcMotor.class, "intake_spin_motor");

        flDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rlDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        downMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        downMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rlDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rrDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        downMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flDrive.setDirection(DcMotor.Direction.FORWARD);
        frDrive.setDirection(DcMotor.Direction.REVERSE);
        rlDrive.setDirection(DcMotor.Direction.FORWARD);
        rrDrive.setDirection(DcMotor.Direction.REVERSE);
        upMotor.setDirection(DcMotor.Direction.FORWARD);
        downMotor.setDirection(DcMotor.Direction.REVERSE);

        int currentUpPos = upMotor.getCurrentPosition();
        int currentDownPos = downMotor.getCurrentPosition();

        upMotor.setTargetPosition(currentUpPos);
        downMotor.setTargetPosition(currentDownPos);

        upMotor.setPower(LiftPower);
        downMotor.setPower(LiftPower);

        markerArm.setPosition(markerArmUp);
        dispServo.setPosition(dispServoDown);
        dispExtensionServo.setPosition(dispExtensionServoIn);
        intakeFlipServo.setPosition(intakeFlipServoUp);

        // IMU gyro setup
        telemetry.addData(">", "WAIT FOR IMU TO CALIBRATE...");
        telemetry.update();

        // Setup for internal IMU data logging - do not touch
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Initialize hardware
        internal_imu = hardwareMap.get(BNO055IMU.class, "internal_imu");
        internal_imu.initialize(parameters);

        telemetry.addData(">", "IMU Calibrated: Press Play to start");
        resetGlobalHeading();

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            intakeFlipServo.setPosition(intakeFlipServoDown);
            dispExtensionServo.setPosition(dispExtensionServoOut);
            // Detach robot from lander
            upMotor.setTargetPosition(currentUpPos + 1300);
            downMotor.setTargetPosition(currentDownPos + 1300);
            upMotor.setPower(LiftPower);
            downMotor.setPower(LiftPower);
            sleep(3000);
            intakeFlipServo.setPosition(intakeFlipServoDown);


            dispServo.setPosition(dispServoUp);

            // Move forward
            encoderDrive(1, 8, 8, 8, 8);
            pos = detector.getXPosition();

            telemetry.addData("X position", detector.getXPosition()); // Gold X position.
            telemetry.update();
            sleep(100);

            // Check if mineral is in right position
            if (pos > 350 && detector.isFound()) {
                // Turn off detector
                detector.disable();
                mineralPos = "right";
                telemetry.addData("Mineral position", mineralPos);
                telemetry.update();
                dispServo.setPosition(dispServoDown);
                // Lower dispenser
                upMotor.setTargetPosition(currentUpPos - 1200);
                downMotor.setTargetPosition(currentDownPos - 1200);
                upMotor.setPower(-LiftPower);
                downMotor.setPower(-LiftPower);
                intakeFlipServo.setPosition(intakeFlipServoDown);
                turnWithImu(36);
                intakeSpinMotor.setPower(1);
                encoderDrive(1, 41, 41, 41, 41);
                intakeSpinMotor.setPower(0);
                intakeFlipServo.setPosition(intakeFlipServoUp);
                //move forward

                encoderDrive(1, 30, 30, 30, 30);
                turnWithImu(-67);
                encoderDrive(1, 20, 20, 20, 20);
                //place marker
                markerArm.setPosition(markerArmDown);
                sleep(1000); //wait for marker to fall down
                markerArm.setPosition(markerArmUp);
                encoderDrive(1, -60, -60, -60, -60);
                turnWithImu(-72);
                encoderDrive(0.6, 150, 150, 150, 150);//drive back to starting point
                turnWithImu(-32);
                encoderDrive(0.6, 30, 30, 30, 30);//drive back to starting point
                intakeFlipServo.setPosition(intakeFlipServoTrueMid);

            }
            //check if mineral is in left position
            else if (pos < 100 && detector.isFound()) {
                //turn off detector
                detector.disable();
                mineralPos = "left";
                telemetry.addData("Mineral position", mineralPos);
                telemetry.update();
                dispServo.setPosition(dispServoDown);
                // Lower dispenser
                upMotor.setTargetPosition(currentUpPos - 1200);
                downMotor.setTargetPosition(currentDownPos - 1200);
                upMotor.setPower(-LiftPower);
                downMotor.setPower(-LiftPower);
                intakeFlipServo.setPosition(intakeFlipServoDown);
                turnWithImu(-32);
                intakeSpinMotor.setPower(1);
                encoderDrive(.6, 55, 55, 55, 55);
                intakeSpinMotor.setPower(0);
                intakeFlipServo.setPosition(intakeFlipServoUp);
                turnWithImu(46);
                encoderDrive(1, 60, 60, 60, 60);
                turnWithImu(26);
                markerArm.setPosition(markerArmDown);
                turnWithImu(170);
                encoderDrive(.6, 100, 100, 100, 100);
                markerArm.setPosition(markerArmUp);
                turnWithImu(47);
                encoderDrive(.6, 60, 60, 60, 60);
                intakeFlipServo.setPosition(intakeFlipServoTrueMid);

            }
            //go to center position
            else {
                //turn off detector
                detector.disable();
                mineralPos = "center";
                telemetry.addData("Mineral position", mineralPos);
                telemetry.update();
                dispServo.setPosition(dispServoDown);
                // Lower dispenser
                upMotor.setTargetPosition(currentUpPos - 1200);
                downMotor.setTargetPosition(currentDownPos - 1200);
                upMotor.setPower(-LiftPower);
                downMotor.setPower(-LiftPower);
                intakeFlipServo.setPosition(intakeFlipServoDown);
                intakeSpinMotor.setPower(1);
                //move forward
                encoderDrive(1, 30, 30, 30, 30);
                encoderDrive(1, 40, 40, 40, 40);
                //place marker
                intakeSpinMotor.setPower(0);
                intakeFlipServo.setPosition(intakeFlipServoUp);
                markerArm.setPosition(markerArmDown);
                sleep(1000);//for markerarm to have enuf time to drop
                markerArm.setPosition(markerArmUp);
                encoderDrive(1, -70, -70, -70, -70);
                turnWithImu(-55);
                encoderDrive(1, 65, 65, 65, 65);
                turnWithImu(-70);
                encoderDrive(0.6, 60, 60, 60, 60);

            }
            detector.disable();
            //wakeLock.release();
            sleep(300000);
        }
    }

    public void encoderDrive(double speed, int flDrivePos, int frDrivePos, int rlDrivePos, int rrDrivePos) {

        flDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rlDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rlDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rrDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        flDrivePos = flDrivePos * (1125 / ((42 / 35) * (32))) + flDrive.getCurrentPosition();
        frDrivePos = frDrivePos * (1125 / ((42 / 35) * (32))) + frDrive.getCurrentPosition();
        rlDrivePos = rlDrivePos * (1125 / ((42 / 35) * (32))) + rlDrive.getCurrentPosition();
        rrDrivePos = rrDrivePos * (1125 / ((42 / 35) * (32))) + rrDrive.getCurrentPosition();

        flDrive.setTargetPosition(flDrivePos);
        frDrive.setTargetPosition(frDrivePos);
        rlDrive.setTargetPosition(rlDrivePos);
        rrDrive.setTargetPosition(rrDrivePos);

        flDrive.setPower(speed);
        frDrive.setPower(speed);
        rlDrive.setPower(speed);
        rrDrive.setPower(speed);

        while (flDrive.isBusy() || frDrive.isBusy() || rlDrive.isBusy() || rrDrive.isBusy()) {

        }

        flDrive.setPower(0);
        frDrive.setPower(0);
        rlDrive.setPower(0);
        rrDrive.setPower(0);

        flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        lastHeading = -Float.parseFloat(formatAngle(lastAngles.angleUnit, lastAngles.firstAngle));
        rawHeading = -Float.parseFloat(formatAngle(rawAngles.angleUnit, rawAngles.firstAngle));
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

    public void stopDrive() {
        flDrive.setPower(0);
        rlDrive.setPower(0);
        frDrive.setPower(0);
        frDrive.setPower(0);
    }

    public void turnWithImu(double degrees) {
        resetGlobalHeading();
        // Turning left
        if (degrees < 0) {
            flDrive.setPower(-0.8); //init turn, QUICKLY
            rlDrive.setPower(-0.8);
            frDrive.setPower(0.8);
            rrDrive.setPower(0.8);
            while (getGlobalHeading() > degrees) {
            }
            if (getGlobalHeading() != degrees) { //final adjustment, SLOWLY
                if (getGlobalHeading() > degrees) {
                    flDrive.setPower(-0.1);
                    rlDrive.setPower(-0.1);
                    frDrive.setPower(0.1);
                    rrDrive.setPower(0.1);
                    while (getGlobalHeading() > degrees) ;
                } else if (getGlobalHeading() < degrees) {
                    flDrive.setPower(0.1);
                    rlDrive.setPower(0.1);
                    frDrive.setPower(-0.1);
                    rrDrive.setPower(-0.1);
                    while (getGlobalHeading() < degrees) ;
                }
            }
        }
        // Turning right
        else if (degrees > 0) {
            flDrive.setPower(0.8); //init turn, QUICKLY
            rlDrive.setPower(0.8);
            frDrive.setPower(-0.8);
            rrDrive.setPower(-0.8);
            while (getGlobalHeading() < degrees) {
            }
            if (getGlobalHeading() != degrees) { //final adjustment, SLOWLY
                if (getGlobalHeading() > degrees) {
                    flDrive.setPower(-0.2);
                    rlDrive.setPower(-0.2);
                    frDrive.setPower(0.2);
                    rrDrive.setPower(0.2);
                    while (getGlobalHeading() > degrees) ;
                } else if (getGlobalHeading() < degrees) {
                    flDrive.setPower(0.2);
                    rlDrive.setPower(0.2);
                    frDrive.setPower(-0.2);
                    rrDrive.setPower(-0.2);
                    while (getGlobalHeading() < degrees) ;
                }
            }
        }
        stopDrive();

    }
}

