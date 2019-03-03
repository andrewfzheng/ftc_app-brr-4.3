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
    // Hardware declaration
    private DcMotor flDrive;
    private DcMotor frDrive;
    private DcMotor rlDrive;
    private DcMotor rrDrive;
    private Servo intakeFlipServo;
    private DcMotor intakeSpinMotor;
    private DcMotor upMotor;
    private DcMotor downMotor;
    private DcMotor inMotor;
    private Servo dispServo;
    private Servo dispExtensionServo;
    private Servo markerArm;
    private Servo sweepServo;

    @Override
    public void runOpMode() {
        // Initialize hardware
        flDrive = hardwareMap.get(DcMotor.class, "fl_drive");
        frDrive = hardwareMap.get(DcMotor.class, "fr_drive");
        rlDrive = hardwareMap.get(DcMotor.class, "rl_drive");
        rrDrive = hardwareMap.get(DcMotor.class, "rr_drive");
        intakeFlipServo = hardwareMap.get(Servo.class, "intake_flip_servo");
        intakeSpinMotor = hardwareMap.get(DcMotor.class, "intake_spin_motor");
        upMotor = hardwareMap.get(DcMotor.class, "up_motor");
        downMotor = hardwareMap.get(DcMotor.class, "down_motor");
        inMotor = hardwareMap.get(DcMotor.class, "in_motor");
        dispServo = hardwareMap.get(Servo.class, "disp_servo");
        dispExtensionServo = hardwareMap.get(Servo.class, "disp_extend_servo");
        markerArm = hardwareMap.get(Servo.class, "marker_servo");
        sweepServo = hardwareMap.get(Servo.class, "sweep_servo");
        // Reset motor encoders
        flDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rlDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        downMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        inMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSpinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSpinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set motor behavior when running
        flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        downMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        inMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Set motor behavior when power received is zero
        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rlDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rrDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        downMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        inMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSpinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set motor directions
        flDrive.setDirection(DcMotor.Direction.REVERSE);
        frDrive.setDirection(DcMotor.Direction.FORWARD);
        rlDrive.setDirection(DcMotor.Direction.REVERSE);
        rrDrive.setDirection(DcMotor.Direction.FORWARD);
        upMotor.setDirection(DcMotor.Direction.FORWARD);
        downMotor.setDirection(DcMotor.Direction.REVERSE);
        inMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeSpinMotor.setDirection(DcMotor.Direction.FORWARD);

        // Variables for hardware
        double intakeFlipServoUp = .85;
        double intakeFlipServoMid = .35;
        double intakeFlipServoDown = 0.10;
        double flDrivePower;
        double frDrivePower;
        double rlDrivePower;
        double rrDrivePower;
        double maxDrivePower;
        double forward; //positive is forward
        double rotate; //positive is clockwise
        boolean isAccelReleased = false;
        boolean isAccelOn = false;
        double dispServoUp = 0.094;
        double dispServoDown = 0.80;
        boolean isDispServoReleased = true;
        boolean isDispServoUp = false;
        double dispExtensionServoOut = 0.11;
        double markerArmUp = 0.6;
        int LiftPower = 1;
        int intakeFlipPos = 0;

        dispExtensionServo.setPosition(dispExtensionServoOut);
        markerArm.setPosition(markerArmUp);

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // Values for driving
            forward = gamepad1.left_stick_y;
            rotate = gamepad1.left_stick_x;

            // Assign power to drivetrain motors
            frDrivePower = forward + rotate;
            flDrivePower = forward - rotate;
            rrDrivePower = forward + rotate;
            rlDrivePower = forward - rotate;

            // Normalize drivetrain power
            maxDrivePower = Math.abs(flDrivePower);
            if (Math.abs(frDrivePower) > maxDrivePower) {
                maxDrivePower = Math.abs(frDrivePower);
            }
            if (Math.abs(rlDrivePower) > maxDrivePower) {
                maxDrivePower = Math.abs(rlDrivePower);
            }
            if (Math.abs(rrDrivePower) > maxDrivePower) {
                maxDrivePower = Math.abs(rrDrivePower);
            }
            if (maxDrivePower > 1) {
                flDrivePower /= maxDrivePower;
                frDrivePower /= maxDrivePower;
                rlDrivePower /= maxDrivePower;
                rrDrivePower /= maxDrivePower;
            }

            // Turbo mode for drivetrain
            if (gamepad1.b) {
                if (isAccelReleased) {
                    isAccelReleased = false;
                    if (isAccelOn) {
                        isAccelOn = false;
                    } else {
                        isAccelOn = true;
                    }
                }
            } else {
                isAccelReleased = true;
            }
            if (isAccelOn) {
                flDrivePower *= 0.6;
                frDrivePower *= 0.6;
                rlDrivePower *= 0.6;
                rrDrivePower *= 0.6;
            }
            // Set drive power
            flDrive.setPower(flDrivePower);
            frDrive.setPower(frDrivePower);
            rlDrive.setPower(rlDrivePower);
            rrDrive.setPower(rrDrivePower);

            // Vertical lift
            if (gamepad2.dpad_down) {
                upMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                downMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                // Negative value to move down
                upMotor.setPower(-LiftPower);
                downMotor.setPower(-LiftPower);
            } else if (gamepad2.dpad_up) {
                upMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                downMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                // Negative value to move down
                upMotor.setPower(LiftPower);
                downMotor.setPower(LiftPower);
            } else {
                upMotor.setPower(0);
                downMotor.setPower(0);
            }

            // Mineral dispenser
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

            // Mineral sweeper on crater-side dispenser
            if (gamepad1.start) {
                sweepServo.setPosition(-1);
            } else {
                sweepServo.setPosition(0.5);
            }

            // Horizontal intake
            if (gamepad2.x) {
                intakeFlipPos = 0;
            } else if (gamepad2.y) {
                intakeFlipPos = 1;
            } else if (gamepad2.b) {
                intakeFlipPos = 2;
            } else if (gamepad1.a) {
                intakeFlipPos = 1;
            }

            if (intakeFlipPos == 0) {
                intakeFlipServo.setPosition(intakeFlipServoDown);
            } else if (intakeFlipPos == 1) {
                intakeFlipServo.setPosition(intakeFlipServoMid);
            } else if (intakeFlipPos == 2) {
                intakeFlipServo.setPosition(intakeFlipServoUp);

            // Horizontal intake retraction
            if (gamepad1.y) {
                inMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                inMotor.setPower(1);
            } else if (gamepad1.x) {
                inMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                inMotor.setPower(-1);
            } else {
                inMotor.setPower(0);
            }

            // Intake collector
            if (gamepad2.left_bumper) {
                intakeSpinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intakeSpinMotor.setPower(-1);
            } else if (gamepad2.right_bumper) {
                intakeSpinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intakeSpinMotor.setPower(1);
            } else {
                intakeSpinMotor.setPower(0);
            }

            // Update telemetry
            telemetry.addData("flDrivePower: ", flDrivePower);
            telemetry.addData("frDrivePower: ", frDrivePower);
            telemetry.addData("rlDrivePower: ", rlDrivePower);
            telemetry.addData("rrDrivePower: ", rrDrivePower);
            telemetry.addData("isAccelOn: ", !isAccelOn);
            telemetry.update();

            }
        }
    }
}



