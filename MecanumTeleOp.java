package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Mecanum TeleOp", group = "TeleOp")
public class MecanumTeleOp extends LinearOpMode {

    // Drive motors (Control Hub): FR = 0, BR = 1, BL = 2, FL = 3
    private DcMotor FR, BR, BL, FL;

    // Expansion Hub: intake = port 0, shooter = port 1
    private DcMotor intake, move, shooter;

    // Power settings
    private static final double SLOW_MODE_SCALE = 0.50;  // Y toggle
    private static final double INTAKE_POWER    = 0.70;  // B mode
    private static final double MOVE_POWER      = 0.70;  // D-pad up toggle
    private static final double SHOOTER_POWER   = 0.68;  // A toggle

    // Toggle states
    private boolean slowMode = false;
    private boolean moveOn = false;
    private boolean blockMode = false; // B: intake normal + move reversed (keeps balls out of shooter)
    private boolean shooterOn = false;

    // Previous button states (for edge detection)
    private boolean lastY = false;
    private boolean lastA = false;
    private boolean lastDpadUp = false;
    private boolean lastB = false;

    @Override
    public void runOpMode() {

        // ---------- Hardware map ----------
        // Names in the Robot Configuration must match these exactly
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FL = hardwareMap.get(DcMotor.class, "FL");

        intake  = hardwareMap.get(DcMotor.class, "intake");
        move    = hardwareMap.get(DcMotor.class, "move");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        // ---------- Directions ----------
        // Typical setup: left side reversed. If the robot drives wrong,
        // flip these (see notes below the code).
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        move.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);

        // ---------- Zero power behavior ----------
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---------- Run modes ----------
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        move.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ================= TOGGLES =================

            // Y: slow mode toggle (50%)
            if (gamepad1.y && !lastY) {
                slowMode = !slowMode;
            }
            lastY = gamepad1.y;

            // A: shooter toggle (68%) only
            if (gamepad1.a && !lastA) {
                shooterOn = !shooterOn;
            }
            lastA = gamepad1.a;

            // D-pad up: move (second intake) toggle (70%)
            if (gamepad1.dpad_up && !lastDpadUp) {
                moveOn = !moveOn;
            }
            lastDpadUp = gamepad1.dpad_up;

            // B: intake normal + move reverse (shooter is NOT affected by B)
            if (gamepad1.b && !lastB) {
                blockMode = !blockMode;
            }
            lastB = gamepad1.b;

            // ================= DRIVE (MECANUM) =================
            double y  = -gamepad1.left_stick_y;   // forward/back (stick up is negative)
            double x  = gamepad1.left_stick_x;    // strafe left/right
            double rx = gamepad1.right_stick_x;   // rotate

            // Slight boost to counter imperfect strafing
            x *= 1.1;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            double flPower = (y + x + rx) / denominator;
            double blPower = (y - x + rx) / denominator;
            double frPower = (y - x - rx) / denominator;
            double brPower = (y + x - rx) / denominator;

            // Apply slow mode
            double scale = slowMode ? SLOW_MODE_SCALE : 1.0;

            FL.setPower(flPower * scale);
            BL.setPower(blPower * scale);
            FR.setPower(frPower * scale);
            BR.setPower(brPower * scale);

            // ================= INTAKE =================
            // D-pad down: while held, reverses ONLY the intake (overrides B mode)
            if (gamepad1.dpad_down) {
                intake.setPower(-INTAKE_POWER);
            } else {
                // intake runs normally while B mode is on
                intake.setPower(blockMode ? INTAKE_POWER : 0);
            }

            // move: D-pad up toggles it on/off (70%). B mode overrides it with reverse.
            // B mode: move runs in reverse so balls don't go into the shooter.
            boolean moveReversed = blockMode;
            if (moveReversed) {
                move.setPower(-MOVE_POWER);
            } else {
                move.setPower(moveOn ? MOVE_POWER : 0);
            }

            // ================= SHOOTER =================
            shooter.setPower(shooterOn ? SHOOTER_POWER : 0);

            // ================= TELEMETRY =================
            telemetry.addData("Slow Mode (Y)", slowMode ? "ON (50%)" : "OFF");
            telemetry.addData("Intake", gamepad1.dpad_down ? "REVERSED" : (blockMode ? "ON (70%)" : "OFF"));
            telemetry.addData("Move (D-pad up)", moveReversed ? "REVERSED" : (moveOn ? "ON (70%)" : "OFF"));
            telemetry.addData("Shooter (A)", shooterOn ? "ON (68%)" : "OFF");
            telemetry.update();
        }

        // Stop everything when the OpMode ends
        FR.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);
        intake.setPower(0);
        move.setPower(0);
        shooter.setPower(0);
    }
}
