package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by andrewzheng on 9/22/18.
 */

@TeleOp(name="TileRunnerTeleOp")
//@Disabled

public class TileRunnerTeleOp extends LinearOpMode {

    //HARDWARE DECLARATION

    private DcMotor flDrive;
    private DcMotor frDrive;
    private DcMotor rlDrive;
    private DcMotor rrDrive;

    private Servo intakeFlipServo;
    private Servo intakeSpinServo;
    private DcMotor upMotor;
    private DcMotor downMotor;
    private DcMotor inMotor;
    private Servo dispServo;
    private Servo markerArm;

    @Override public void runOpMode(){

        //INIT HARDWARE

        flDrive = hardwareMap.get(DcMotor.class, "fl_drive");
        frDrive = hardwareMap.get(DcMotor.class, "fr_drive");
        rlDrive = hardwareMap.get(DcMotor.class, "rl_drive");
        rrDrive = hardwareMap.get(DcMotor.class, "rr_drive");
        intakeFlipServo = hardwareMap.get(Servo.class, "intake_flip_servo");
        intakeSpinServo = hardwareMap.get(Servo.class, "intake_spin_servo");
        upMotor = hardwareMap.get(DcMotor.class, "up_motor");
        downMotor = hardwareMap.get(DcMotor.class, "down_motor");
        inMotor = hardwareMap.get(DcMotor.class, "in_motor");
        dispServo = hardwareMap.get(Servo.class, "disp_servo");
        markerArm = hardwareMap.get(Servo.class, "marker_servo");

        flDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rlDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rlDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rrDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flDrive.setDirection(DcMotor.Direction.REVERSE);
        frDrive.setDirection(DcMotor.Direction.FORWARD);
        rlDrive.setDirection(DcMotor.Direction.REVERSE);
        rrDrive.setDirection(DcMotor.Direction.FORWARD);

        upMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        downMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        inMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        upMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        downMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        inMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        upMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        downMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        upMotor.setDirection(DcMotor.Direction.FORWARD);
        downMotor.setDirection(DcMotor.Direction.REVERSE);
        inMotor.setDirection(DcMotor.Direction.FORWARD);

        //VARIABLES FOR HARDWARE
        double flDrivePower;
        double frDrivePower;
        double rlDrivePower;
        double rrDrivePower;
        double maxDrivePower;

        double forward; //positive is forward
        double rotate; //positive is clockwise

        int currentHPos = 0;
        int currentVUPPos = upMotor.getCurrentPosition();
        int currentVDOWNPos = downMotor.getCurrentPosition();
        boolean isVPositionHolding;
        boolean isHPositionHolding;

        boolean isAccelReleased = false;
        boolean isAccelOn = true;

        double dispServoUp = 0.85;
        double dispServoDown = 0.2;
        boolean isDispServoReleased = true;
        boolean isDispServoUp = false;

        double markerArmUp = 0.6;
        double markerArmDown = 0.07;
        boolean isMarkerReleased = true;
        boolean isMarkerUp = true;

        double intakeFlipServoUp = 0.92;
        double intakeFlipServoMid = 0.35;
        double instakeFlipServoDown = 0.19;
        boolean isIntakeFlipReleased = true;
        boolean isIntakeFlipMidReleased = true;
        boolean isIntakeFlipUp = true;
        boolean isIntakeFlipMid = true;


        int currentUpPos = upMotor.getCurrentPosition();
        int currentDownPos = downMotor.getCurrentPosition();

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        //boolean isCollectorExtended = false;
        
        waitForStart();

        while (opModeIsActive()){

            //ITERATIVE CODE
            forward = gamepad1.left_stick_y;
            //strafe = gamepad1.right_stick_x;
            rotate = gamepad1.left_stick_x;

            frDrivePower = forward + rotate;
            flDrivePower = forward - rotate;
            rrDrivePower = forward + rotate;
            rlDrivePower = forward - rotate;

            //normalize mecanum drive
            maxDrivePower = Math.abs(flDrivePower);
            if (Math.abs(frDrivePower) > maxDrivePower){
                maxDrivePower = Math.abs(frDrivePower);
            }
            if (Math.abs(rlDrivePower) > maxDrivePower){
                maxDrivePower = Math.abs(rlDrivePower);
            }
            if (Math.abs(rrDrivePower) > maxDrivePower){
                maxDrivePower = Math.abs(rrDrivePower);
            }
            if (maxDrivePower > 1) {
                flDrivePower /= maxDrivePower;
                frDrivePower /= maxDrivePower;
                rlDrivePower /= maxDrivePower;
                rrDrivePower /= maxDrivePower;
            }

            //turbo mode
            if (gamepad1.a) {
                if (isAccelReleased) {
                    isAccelReleased = false;
                    if (isAccelOn){
                        isAccelOn = false;
                    }
                    else{
                        isAccelOn = true;
                    }
                }
            }
            else {
                isAccelReleased = true;
            }

            if (isAccelOn){
                flDrivePower *= 0.6;
                frDrivePower *= 0.6;
                rlDrivePower *= 0.6;
                rrDrivePower *= 0.6;
            }

            //marker arm
            if (gamepad1.b || gamepad2.b) { //both drivers can control marker arm
                if (isMarkerReleased) {
                    isMarkerReleased = false;
                    if (isMarkerUp) {
                        isMarkerUp = false;
                    } else {
                        isMarkerUp = true;
                    }
                }
            } else {
                isMarkerReleased = true;
            }

            if (isMarkerUp) {
                markerArm.setPosition(markerArmDown);
            } else {
                markerArm.setPosition(markerArmUp);
            }

            // vertical lift
            if (gamepad2.dpad_down) {
                upMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                upMotor.setPower(-1); //negative value to move down
                downMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                downMotor.setPower(-1); //negative value to move down
                isVPositionHolding = false;
                currentVUPPos = upMotor.getCurrentPosition();
                currentVDOWNPos = upMotor.getCurrentPosition();
            } else if (gamepad2.dpad_up) {
                upMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                upMotor.setPower(1); //positive value to move up
                downMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                downMotor.setPower(1); //positive value to move up
                isVPositionHolding = false;
                currentVDOWNPos = upMotor.getCurrentPosition();
                currentVUPPos = upMotor.getCurrentPosition();
            }
            else {
                upMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                upMotor.setTargetPosition(currentVUPPos);
                downMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                downMotor.setTargetPosition(currentVDOWNPos);
                isVPositionHolding = true;
            }

            //vertical dispenser (dump on top)
            if (gamepad2.a) {
                if (isDispServoReleased) {
                    isDispServoReleased = false;
                    if (isDispServoUp) {
                        isDispServoUp = false;
                    } else {
                        isDispServoUp = true;
                    }
                }
            } else {
                isDispServoReleased = true;
            }

            if (isDispServoUp) {
                dispServo.setPosition(dispServoUp);
            } else {
                dispServo.setPosition(dispServoDown);
            }

            //horizontal intake flip up-down
            if (gamepad2.x) {
                if (isIntakeFlipReleased) {
                    isIntakeFlipReleased = false;
                    if (isIntakeFlipUp) {
                        isIntakeFlipUp = false;
                    } else {
                        isIntakeFlipUp = true;
                    }
                }
            } else {
                isIntakeFlipReleased = true;
            }

            if (isIntakeFlipUp) {
                intakeFlipServo.setPosition(intakeFlipServoUp);
            } else {
                intakeFlipServo.setPosition(instakeFlipServoDown);
            }

            //horizontal intake flip to middle
            if (gamepad2.y) {
                intakeFlipServo.setPosition(intakeFlipServoMid);
            }

            //horizontal intake retraction
            if (-gamepad2.right_stick_y != 0) { //flipped y input because controller input is switched
                inMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                inMotor.setPower(-gamepad2.right_stick_y); //no need to readjust up or down power because using ENCODERS
                isHPositionHolding = false;
                currentHPos = inMotor.getCurrentPosition();
            }
            else {
                inMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                inMotor.setTargetPosition(currentHPos);
                isHPositionHolding = true;
            }

            //activate collector only if bumpers are pressed
            if (gamepad2.left_bumper) {
                intakeSpinServo.setPosition(1);
            }
            //activate collector only if bumpers are pressed
            else if (gamepad2.right_bumper) {
                intakeSpinServo.setPosition(0);
            }
            //stop intake
            else {
                intakeSpinServo.setPosition(0.5);
            }

            //set drive power
            flDrive.setPower(flDrivePower);
            frDrive.setPower(frDrivePower);
            rlDrive.setPower(rlDrivePower);
            rrDrive.setPower(rrDrivePower);

            //update telemetry
            telemetry.addData("flDrivePower: ", flDrivePower);
            telemetry.addData("frDrivePower: ", frDrivePower);
            telemetry.addData("rlDrivePower: ", rlDrivePower);
            telemetry.addData("rrDrivePower: ", rrDrivePower);
            telemetry.addData("Vertical Lift position holding? ", isVPositionHolding);
            telemetry.addData("Horizontal Lift position holding? ", isHPositionHolding);
            telemetry.addData("isAccelOn: ", isAccelOn);
            telemetry.update();
        }
    }
}



