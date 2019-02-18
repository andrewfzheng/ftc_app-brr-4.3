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

@Autonomous(name = "IMUCraterAuto", group = "DogeCV")


public class IMUCraterAuto extends LinearOpMode {

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
    private Servo sweepServo;


    // Variables for hardware
    double intakeFlipServoUp = .92;
    double intakeFlipServoLowMid = 0.61;
    double intakeFlipServoDown = 0.15;
    double intakeFlipServoTrueMid = 0.35;

    double dispServoUp = 0.094 ;
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
        sweepServo = hardwareMap.get(Servo.class, "sweep_servo");

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
        telemetry.addData("IMU Status", "Waiting for IMU to Calibrate");
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

        telemetry.addData("IMU Status", "IMU Calibrated - Press Play to start");
        resetGlobalHeading();

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            dispExtensionServo.setPosition(dispExtensionServoOut);
            // Detach robot from lander
            //upMotor.setTargetPosition(currentUpPos + 1300);
            //downMotor.setTargetPosition(currentDownPos + 1300);
            //upMotor.setPower(LiftPower);
            //downMotor.setPower(LiftPower);
            sleep(3000);
            // Flip dispenser up
            dispServo.setPosition(dispServoUp);
            // Move forward
            encoderDrive(1, 8, 8, 8, 8);
            // Flip intake down
            intakeFlipServo.setPosition(intakeFlipServoDown);
            // Get position of largest yellow object in camera FOV
            pos = detector.getXPosition();
            telemetry.addData("X position", detector.getXPosition()); // Gold X position.
            telemetry.update();
            // Check if mineral is in right position
            if (pos > 350 && detector.isFound()) {
                // Turn off detector
                detector.disable();
                mineralPos = "right";
                telemetry.addData("Mineral position", mineralPos);
                telemetry.update();
                // Turn right
                turnWithImu(13);
                // Activate intake
                intakeSpinMotor.setPower(-1);
                // Move forward
                encoderDrive(1, 43, 43, 43, 43);
                // move backward
                encoderDrive(1, -42, -42, -42, -42);
                // Stop intake
                intakeSpinMotor.setPower(0);
                // Turn left
                turnWithImu(-50);
                // Move forward
                encoderDrive(1, 70, 70, 70, 70);
                // Turn left
                turnWithImu(-60);
                // Move forward
                encoderDrive(1, 100, 100, 100, 100);
                // Turn right
                turnWithImu(6);
                // Place marker
                markerArm.setPosition(markerArmDown);
                // Move backward
                encoderDrive(1, -3, -3, -3, -3);
                // Turn left
                turnWithImu(-9);
                // Move back
                encoderDrive(0.6, -120, -120, -120, -120);
                encoderDrive(1, -43, -43, -43, -43);

                resetGlobalHeading();
                flDrive.setPower(-0.8);
                rlDrive.setPower(-0.8);
                frDrive.setPower(0.8);
                rrDrive.setPower(0.8);
                while (getGlobalHeading() < 100) {

                }
                stopDrive();

                encoderDrive(1, 75, 75, 75, 75);

                resetGlobalHeading();
                flDrive.setPower(-0.8);
                rlDrive.setPower(-0.8);
                frDrive.setPower(0.8);
                rrDrive.setPower(0.8);
                while (getGlobalHeading() < 40) {

                }
                stopDrive();

                encoderDrive(1, 50, 50, 50, 50);

                markerArm.setPosition(0.07);
                //DRIVE TO CRATER PARK
                encoderDrive(1, -120, -120, -120, -120);
            }
            // Check if mineral is in left position
            else if (pos < 100 && detector.isFound()) {
                // Turn off detector
                detector.disable();
                mineralPos = "left";
                telemetry.addData("Mineral position", mineralPos);
                telemetry.update();
                // Turn left
                turnWithImu(-11);
                // Activate intake
                intakeSpinMotor.setPower(-1);
                // Move forward
                encoderDrive(1, 38, 38, 38, 38);
                // Move backward
                encoderDrive(1, -37, -37, -37, -37);
                // Stop intake
                intakeSpinMotor.setPower(0);
                // Turn left
                turnWithImu(-20);
                // Move forward
                encoderDrive(1, 70, 70, 70, 70);
                turnWithImu(-53);
                // Move forward
                intakeFlipServo.setPosition(intakeFlipServoUp);

                //turn left
                resetGlobalHeading();
                flDrive.setPower(-0.8);
                rlDrive.setPower(-0.8);
                frDrive.setPower(0.8);
                rrDrive.setPower(0.8);
                while (getGlobalHeading() < 10) {

                }
                stopDrive();
                //move forward
                encoderDrive(1, 38, 38, 38, 38);
                //move backward
                encoderDrive(1, -38, -38, -38, -38);
                //turn left
                resetGlobalHeading();
                flDrive.setPower(-0.8);
                rlDrive.setPower(-0.8);
                frDrive.setPower(0.8);
                rrDrive.setPower(0.8);
                while (getGlobalHeading() < 20) {

                }
                stopDrive();
                //move forward
                encoderDrive(1, 79, 79, 79, 79);
                //turn left
                resetGlobalHeading();
                flDrive.setPower(-0.8);
                rlDrive.setPower(-0.8);
                frDrive.setPower(0.8);
                rrDrive.setPower(0.8);
                while (getGlobalHeading() < 54) {

                }
                stopDrive();
                //move forward
                encoderDrive(1, 100, 100, 100, 100);
                turnWithImu(6);
                // Place marker
                markerArm.setPosition(markerArmDown);
                // Move backward
                encoderDrive(1, -3, -3, -3, -3);
                // Turn left
                turnWithImu(-9);
                // Move backward
                encoderDrive(0.6, -145, -145, -145, -145);
                //turn left
                resetGlobalHeading();
                flDrive.setPower(-0.8);
                rlDrive.setPower(-0.8);
                frDrive.setPower(0.8);
                rrDrive.setPower(0.8);
                while (getGlobalHeading() < 1.5) {

                }
                stopDrive();
                //move backward
                encoderDrive(1, -145, -145, -145, -145);
                sleep(300000);
            }
            // Go to center position
            else {
                // Turn off detector
                detector.disable();
                mineralPos = "center";
                telemetry.addData("Mineral position", mineralPos);
                telemetry.update();
                // Lower mineral dispenser
                //upMotor.setTargetPosition(currentUpPos - 1200);
                //downMotor.setTargetPosition(currentDownPos - 1200);
                //upMotor.setPower(-LiftPower);
                //downMotor.setPower(-LiftPower);
                //sleep(1000);
                // Activate intake
                intakeSpinMotor.setPower(-1);
                // Move forward
                encoderDrive(1, 30, 30, 30, 30);
                sleep(3000);
                // Lift intake and eject mineral
                //intakeFlipServo.setPosition(intakeFlipServoUp);
                // Move backward
                encoderDrive(1, -29, -29, -29, -29);
                // Stop intake
                intakeSpinMotor.setPower(0);
                // Lower intake
                //intakeFlipServo.setPosition(intakeFlipServoDown);
                // Raise mineral dispener
                //upMotor.setTargetPosition(currentUpPos + 1300);
                //downMotor.setTargetPosition(currentDownPos + 1300);
                //upMotor.setPower(LiftPower);
                //downMotor.setPower(LiftPower);
                //sleep(3000);
                // Activate sweeper
                //sweepServo.setPosition(-1);
                // Dispense mineral
                //dispServo.setPosition(dispServoUp);
                //sleep(1000);
                // Lower dispenser
                //dispServo.setPosition(dispServoDown);
                // Turn left
                turnWithImu(-35);
                // Move forward
                encoderDrive(1, 70, 70, 70, 70);
                // Turn left
                turnWithImu(-62);
                // Move forward

                /////SPIN INTAKE (it's already down)

                //move forward
                encoderDrive(1, 30, 30, 30, 30);
                sleep(3000);
                //move backward
                encoderDrive(1, -30, -30, -30, -30);

                //////STOP SPIN INTAKE
                intakeFlipServo.setPosition(intakeFlipServoUp);

                runtime.reset();
                //turn left
                resetGlobalHeading();
                flDrive.setPower(-0.8);
                rlDrive.setPower(-0.8);
                frDrive.setPower(0.8);
                rrDrive.setPower(0.8);
                while (getGlobalHeading() < 35) {

                }
                stopDrive();
                //move forward
                encoderDrive(1, 79, 79, 79, 79);
                //turn left
                resetGlobalHeading();
                flDrive.setPower(-0.8);
                rlDrive.setPower(-0.8);
                frDrive.setPower(0.8);
                rrDrive.setPower(0.8);
                while (getGlobalHeading() < 65) {

                }
                stopDrive();
                //move forward
                encoderDrive(1, 100, 100, 100, 100);
                // Turn right
                turnWithImu(6);
                // Place marker
                markerArm.setPosition(markerArmDown);
                // Move backward
                encoderDrive(1, -3, -3, -3, -3);
                // Turn left
                turnWithImu(-9);
                // Move backward
                encoderDrive(0.6, -140, -140, -140, -140);
                resetGlobalHeading();
                flDrive.setPower(-0.8);
                rlDrive.setPower(-0.8);
                frDrive.setPower(0.8);
                rrDrive.setPower(0.8);
                while (getGlobalHeading() < 1.5) {

                }
                stopDrive();

                //move backward
                encoderDrive(1, -140, -154, -140, -140);
            }
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
        // Must reset angle every 360 degrees
        // Positive to right, negative to left
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
        // Turn left
        if (degrees < 0) {
            flDrive.setPower(-0.8);
            rlDrive.setPower(-0.8);
            frDrive.setPower(0.8);
            rrDrive.setPower(0.8);
            while (getGlobalHeading() > degrees) {
            }
        }
        // Turn right
        else {
            flDrive.setPower(0.8);
            rlDrive.setPower(0.8);
            frDrive.setPower(-0.8);
            rrDrive.setPower(-0.8);
            while (getGlobalHeading() < degrees) {
            }
            //Stop all drive motors
        }
        stopDrive();
    }
}

