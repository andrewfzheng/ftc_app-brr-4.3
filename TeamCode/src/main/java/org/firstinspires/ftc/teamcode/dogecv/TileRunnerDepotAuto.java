package org.firstinspires.ftc.teamcode.dogecv;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TileRunnerDepotAuto", group="DogeCV")

public class TileRunnerDepotAuto extends LinearOpMode {

    //VARIABLES FOR HARDWARE
    double intakeFlipServoUp = .85;
    double intakeFlipServoMid = 0.61;
    double intakeFlipServoDown = 0.15;

    int LiftPower = 1;
    double pos = 0;
    double dispServoUp = 0.0976;
    double dispServoDown = .75;
    double markerArmUp = 0.6;
    double markerArmDown = 0.07;
    double dispExtensionServoIn = 0.67;
    double dispExtensionServoOut = 0.11;

    // Detector object
    private GoldAlignDetector detector;
    private ElapsedTime runtime = new ElapsedTime();
    String mineralPos = "none";

    DcMotor upMotor;
    DcMotor downMotor;
    DcMotor flDrive;
    DcMotor frDrive;
    DcMotor rlDrive;
    DcMotor rrDrive;
    Servo markerArm;
    Servo dispServo;
    Servo intakeFlipServo;
    Servo dispExtensionServo;

    @Override
    public void runOpMode() throws InterruptedException{

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

        waitForStart();

        while (opModeIsActive()) {
            dispExtensionServo.setPosition(dispExtensionServoOut);
            intakeFlipServo.setPosition(intakeFlipServoDown);
            //detach robot from lander
            upMotor.setTargetPosition(currentUpPos + 1300);
            downMotor.setTargetPosition(currentDownPos + 1300);
            upMotor.setPower(LiftPower);
            downMotor.setPower(LiftPower);
            sleep(3000);
            dispServo.setPosition(dispServoUp);

            //move forward
            encoderDrive(1, 8, 8, 8, 8);
            //start vuforia
            pos = detector.getXPosition();

            telemetry.addData("X position" , detector.getXPosition()); // Gold X position.
            telemetry.update();

            //check if mineral is in right position
            if (pos > 350 && detector.isFound()) {

                detector.disable();
                mineralPos = "right";
                telemetry.addData("Mineral position", mineralPos);
                telemetry.update();
                intakeFlipServo.setPosition(intakeFlipServoUp);

                //turn right
                encoderDrive(0.6, 16, -16,16, -16);
                //move forward
                encoderDrive(1, 80, 80, 80, 80);
                //turn left
                encoderDrive(0.6, -35, 35, -35, 35);
                //move forward
                encoderDrive(1, 40, 40, 40, 40);
                //place marker
                markerArm.setPosition(markerArmDown);
                //move backward
                encoderDrive(1, -10,-10,-10,-10);
                //turn left
                encoderDrive(0.6, -25, 25,-25, 25);
                //move forward
                encoderDrive(1, 32, 32, 32, 32);
                //turn left
                encoderDrive(0.6, -24,24,-24, 24);
                //lift arm
                markerArm.setPosition(markerArmUp);
                //move forward
                encoderDrive(1, 130, 130, 130, 130);
                //drop collector
                intakeFlipServo.setPosition(intakeFlipServoMid);
            }

            // check if mineral is in left position
            else if (pos < 130 && detector.isFound()) {

                detector.disable();
                mineralPos = "left";
                telemetry.addData("Mineral position", mineralPos);
                telemetry.update();
                intakeFlipServo.setPosition(intakeFlipServoUp);

                //turn left
                encoderDrive(0.6, -16, 16, -16, 16);
                //move forward
                encoderDrive(1, 75, 75, 75, 75);
                //turn right
                encoderDrive(1, 38, -38, 38, -38);
                //move forward
                encoderDrive(1, 50, 50, 50, 50);
                //place marker
                markerArm.setPosition(markerArmDown);
                //move backward
                encoderDrive(1, -60,-60,-60,-60);
                //turn right
                encoderDrive(0.6, 100, -100, 100, -100);
                //lift arm
                markerArm.setPosition(markerArmUp);
                //move forward
                encoderDrive(1,110, 110, 110, 110);
                //drop collector
                intakeFlipServo.setPosition(intakeFlipServoMid);
            }
            //go to center position
            else {

                detector.disable();
                mineralPos = "center";
                telemetry.addData("Mineral position", mineralPos);
                telemetry.update();
                intakeFlipServo.setPosition(intakeFlipServoUp);

                //move forward
                encoderDrive(1, 77, 77, 77, 77);
                //place marker
                markerArm.setPosition(markerArmDown);
                //move backward
                encoderDrive(1, -60, -60, -60, -60);
                //turn left
                encoderDrive(0.6, -41, 41, -41, 41);
                //lift arm
                markerArm.setPosition(markerArmUp);
                //move forward
                encoderDrive(0.6, 90, 90, 90, 90);
                //turn left
                encoderDrive(0.6, -31, 31, -31, 31);
                //move forward
                encoderDrive(1, 30, 30, 30, 30);
                //drop collector
                intakeFlipServo.setPosition(intakeFlipServoMid);
            }
            detector.disable();
            sleep(300000);
        }
    }

        public void encoderDrive ( double speed, int flDrivePos, int frDrivePos, int rlDrivePos, int rrDrivePos) {

        flDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rlDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rlDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rrDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        flDrivePos = flDrivePos * (1125 / ((45 / 35) * (32))) + flDrive.getCurrentPosition();
        frDrivePos = frDrivePos * (1125 / ((45 / 35) * (32))) + frDrive.getCurrentPosition();
        rlDrivePos = rlDrivePos * (1125 / ((45 / 35) * (32))) + rlDrive.getCurrentPosition();
        rrDrivePos = rrDrivePos * (1125 / ((45 / 35) * (32))) + rrDrive.getCurrentPosition();

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
}