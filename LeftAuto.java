package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "LeftAuto", group = "Auto")
public class LeftAuto extends LinearOpMode {

    // Drive motors (Control Hub): FR = 0, BR = 1, BL = 2, FL = 3
    private DcMotor FR, BR, BL, FL;

    // Expansion Hub: intake = port 0, shooter = port 1 (move = its own port)
    private DcMotor intake, move, shooter;

    // ================= ENCODER SETTINGS (CHANGE THESE TO MATCH YOUR ROBOT) =================
    // Defaults assume goBILDA 435 RPM motors (13.7:1) and 96 mm mecanum wheels.
    private static final double COUNTS_PER_MOTOR_REV = 383.6;  // encoder ticks per motor output rev
    private static final double GEAR_REDUCTION       = 1.0;    // external gearing (1.0 = direct drive)
    private static final double WHEEL_DIAMETER_IN    = 3.78;   // 96 mm = 3.78 inches
    private static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * GEAR_REDUCTION) / (WHEEL_DIAMETER_IN * Math.PI);

    // Mecanum wheels slip when strafing, so strafing needs extra distance.
    // Tune this: if it strafes too short, raise it; too far, lower it.
    private static final double STRAFE_CORRECTION = 1.1;

    private static final double DRIVE_POWER = 0.5;

    @Override
    public void runOpMode() {

        // ---------- Hardware map ----------
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FL = hardwareMap.get(DcMotor.class, "FL");

        intake  = hardwareMap.get(DcMotor.class, "intake");
        move    = hardwareMap.get(DcMotor.class, "move");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        // ---------- Directions (same as TeleOp) ----------
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        move.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Mechanisms stay off during this auto
        intake.setPower(0);
        move.setPower(0);
        shooter.setPower(0);

        telemetry.addData("Status", "Ready - LeftAuto");
        telemetry.update();

        waitForStart();

        // ================= PATH =================
        driveForward(24);     // forward 2 feet
        strafeRight(21);      // strafe right 1.75 feet

        telemetry.addData("Status", "Done");
        telemetry.update();
    }

    // ================= HELPERS (inches) =================

    /** Drive forward (positive) or backward (negative). */
    private void driveForward(double inches) {
        move4(inches, inches, inches, inches);
    }

    /** Strafe right (positive) or left (negative). */
    private void strafeRight(double inches) {
        double d = inches * STRAFE_CORRECTION;
        // Strafe right: FL +, BL -, FR -, BR +
        move4(d, -d, -d, d);
    }

    /** Move each wheel a set number of inches, all arriving together. */
    private void move4(double fl, double bl, double fr, double br) {
        if (!opModeIsActive()) return;

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setTargetPosition((int) Math.round(fl * COUNTS_PER_INCH));
        BL.setTargetPosition((int) Math.round(bl * COUNTS_PER_INCH));
        FR.setTargetPosition((int) Math.round(fr * COUNTS_PER_INCH));
        BR.setTargetPosition((int) Math.round(br * COUNTS_PER_INCH));

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FL.setPower(DRIVE_POWER);
        BL.setPower(DRIVE_POWER);
        FR.setPower(DRIVE_POWER);
        BR.setPower(DRIVE_POWER);

        while (opModeIsActive() && (FL.isBusy() || BL.isBusy() || FR.isBusy() || BR.isBusy())) {
            telemetry.addData("FL", FL.getCurrentPosition() + " / " + FL.getTargetPosition());
            telemetry.addData("BL", BL.getCurrentPosition() + " / " + BL.getTargetPosition());
            telemetry.addData("FR", FR.getCurrentPosition() + " / " + FR.getTargetPosition());
            telemetry.addData("BR", BR.getCurrentPosition() + " / " + BR.getTargetPosition());
            telemetry.update();
        }

        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(150); // short settle between moves
    }
}
