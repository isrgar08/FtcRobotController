package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "TeleOpFull", group = "Drive")
public class TeleOpFull extends LinearOpMode {

    // ----------------------------
    // 1) Drivetrain motors
    // ----------------------------
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    // ----------------------------
    // 2) Ball path / turret hardware
    // ----------------------------
    DcMotor intake;        // pulls balls in or spits out
    DcMotor placeholder;   // moves balls up toward turret
    DcMotor shooter1;      // first shooter motor
    DcMotor shooter2;      // second shooter motor

    CRServo indexer;       // CR servo that spins balls around disc
    CRServo servo1;        // CR servo for turret rotation (yaw)
    CRServo servo2;        // CR servo for hood pitch (angle)

    // ----------------------------
    // 3) Tuning constants
    // ----------------------------

    // Overall drive speed (change this to make robot faster/slower)
    double DRIVE_SPEED_SCALE = 0.7;

    // Shooter base power (change this at competition to tune shot)
    double SHOOTER_BASE_POWER = 0.8;   // between 0.0 and 1.0

    // Deadzones so tiny stick movement doesn't move things
    double JOYSTICK_DEADZONE = 0.05;
    double SERVO_DEADZONE = 0.10;

    // Optional scaling for CR servo speeds (rotation / hood / indexer)
    double INDEXER_SCALE = 1.0;  // 1.0 = full speed
    double SERVO1_SCALE  = 1.0;
    double SERVO2_SCALE  = 1.0;

    @Override
    public void runOpMode() {

        // ============================================================
        // HARDWARE MAPPING
        // Robot Configuration *must* use these exact names:
        // leftFront, leftBack, rightFront, rightBack
        // intake, placeholder, shooter1, shooter2
        // indexer, servo1, servo2
        // ============================================================

        // Drivetrain
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        // Ball path / turret
        intake      = hardwareMap.get(DcMotor.class, "intake");
        placeholder = hardwareMap.get(DcMotor.class, "placeholder");
        shooter1    = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2    = hardwareMap.get(DcMotor.class, "shooter2");

        indexer = hardwareMap.get(CRServo.class, "indexer");
        servo1  = hardwareMap.get(CRServo.class, "servo1");
        servo2  = hardwareMap.get(CRServo.class, "servo2");

        // ============================================================
        // MOTOR / SERVO DIRECTIONS
        // Adjust here ONLY if robot moves backwards or mechanisms
        // run the opposite way you expect.
        // ============================================================

        // Drivetrain: these directions make "forward" on stick = robot forward
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Intake motor is wired backwards:
        // we set REVERSE so that +power = balls go IN
        intake.setDirection(DcMotor.Direction.REVERSE);

        // Placeholder and shooters: start FORWARD
        // (If balls move wrong way at the field, flip these later.)
        placeholder.setDirection(DcMotor.Direction.FORWARD);
        shooter1.setDirection(DcMotor.Direction.FORWARD);
        shooter2.setDirection(DcMotor.Direction.FORWARD);

        // CR servos: default FORWARD. If sticks feel backwards,
        // we'll flip the sign in the control section instead.
        indexer.setDirection(CRServo.Direction.FORWARD);
        servo1.setDirection(CRServo.Direction.FORWARD);
        servo2.setDirection(CRServo.Direction.FORWARD);

        // ============================================================
        // BASIC MOTOR SETTINGS
        // ============================================================

        // Brake when power = 0 so robot/coasters stop quickly
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        placeholder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Run without encoders for TeleOp
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        placeholder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("TeleOpFull initialized. Ready to start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ========================================================
            // SECTION A: GAMEPAD 1 DRIVETRAIN (MECANUM DRIVE)
            // Controls:
            //  - left stick:  forward/back + strafe
            //  - right stick: turn
            // If you want to tweak driving, this is the main place.
            // ========================================================

            double y  = -gamepad1.left_stick_y;   // forward/back
            double x  =  gamepad1.left_stick_x;   // strafe
            double rx =  gamepad1.right_stick_x;  // rotate

            // Deadzones to avoid drift
            if (Math.abs(y)  < JOYSTICK_DEADZONE) y  = 0;
            if (Math.abs(x)  < JOYSTICK_DEADZONE) x  = 0;
            if (Math.abs(rx) < JOYSTICK_DEADZONE) rx = 0;

            // Mecanum wheel calculations
            double leftFrontPower  = y + x + rx;
            double leftBackPower   = y - x + rx;
            double rightFrontPower = y - x - rx;
            double rightBackPower  = y + x - rx;

            // Normalize power so no wheel gets more than 1.0
            double max = Math.max(1.0,
                    Math.max(Math.abs(leftFrontPower),
                            Math.max(Math.abs(leftBackPower),
                                    Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower)))));

            leftFrontPower  /= max;
            leftBackPower   /= max;
            rightFrontPower /= max;
            rightBackPower  /= max;

            // Global driving speed scale (adjust DRIVE_SPEED_SCALE at top)
            leftFrontPower  *= DRIVE_SPEED_SCALE;
            leftBackPower   *= DRIVE_SPEED_SCALE;
            rightFrontPower *= DRIVE_SPEED_SCALE;
            rightBackPower  *= DRIVE_SPEED_SCALE;

            // Apply to motors
            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);

            // ========================================================
            // SECTION B: INTAKE CONTROL (G1 PRIORITY OVER G2)
            //
            //  - Gamepad1 A: intake IN (balls in)  at 100%
            //  - Gamepad1 B: intake OUT (spit)    at 100%
            //  - Gamepad2 A/B: only used if G1 is NOT pressing A or B
            //  Priority logic: G1 wins if both are trying to control.
            // ========================================================

            double intakePower = 0.0;

            // --- Gamepad 1 has priority ---
            if (gamepad1.a && !gamepad1.b) {
                intakePower = 1.0;    // IN
            } else if (gamepad1.b && !gamepad1.a) {
                intakePower = -1.0;   // OUT
            } else {
                // Gamepad 1 not commanding intake -> check Gamepad 2
                if (gamepad2.a && !gamepad2.b) {
                    intakePower = 1.0;    // IN
                } else if (gamepad2.b && !gamepad2.a) {
                    intakePower = -1.0;   // OUT
                } else {
                    intakePower = 0.0;    // no buttons -> stop
                }
            }

            intake.setPower(intakePower);

            // ========================================================
            // SECTION C: SHOOTERS (GAMEPAD 2, RIGHT TRIGGER)
            //
            //  - RT value (0..1) scales between 0 and SHOOTER_BASE_POWER.
            //  - Both shooter1 and shooter2 always get the same power.
            //  To reduce or increase power, change SHOOTER_BASE_POWER
            //  at the top of this file.
            // ========================================================

            double rt = gamepad2.right_trigger;  // 0..1
            double shooterPower = rt * SHOOTER_BASE_POWER;

            // If you discover the shooter spins the wrong way,
            // you can either flip motor direction in "MOTOR DIRECTIONS"
            // or just multiply by -1 here:
            // shooterPower = -shooterPower;

            shooter1.setPower(shooterPower);
            shooter2.setPower(shooterPower);

            // ========================================================
            // SECTION D: INDEXER (GAMEPAD 2 RIGHT STICK Y)
            //
            //  - Up/down on G2 right stick moves balls around disc.
            //  - Power is proportional to stick value.
            //  If rotation feels backwards, flip the sign.
            // ========================================================

            double indexerInput = -gamepad2.right_stick_y; // up = forward
            if (Math.abs(indexerInput) < SERVO_DEADZONE) indexerInput = 0.0;
            double indexerPower = indexerInput * INDEXER_SCALE;
            indexer.setPower(indexerPower);

            // ========================================================
            // SECTION E: TURRET ROTATION (servo1) - G2 RIGHT STICK X
            //
            //  - Left/right on G2 right stick rotates turret.
            // ========================================================

            double servo1Input = gamepad2.right_stick_x;
            if (Math.abs(servo1Input) < SERVO_DEADZONE) servo1Input = 0.0;
            double servo1Power = servo1Input * SERVO1_SCALE;
            servo1.setPower(servo1Power);

            // ========================================================
            // SECTION F: HOOD PITCH (servo2) - G2 LEFT STICK Y
            //
            //  - Up/down on G2 left stick adjusts hood angle.
            // ========================================================

            double servo2Input = -gamepad2.left_stick_y; // up = positive
            if (Math.abs(servo2Input) < SERVO_DEADZONE) servo2Input = 0.0;
            double servo2Power = servo2Input * SERVO2_SCALE;
            servo2.setPower(servo2Power);

            // ========================================================
            // SECTION G: TELEMETRY (OPTIONAL)
            // You can comment out lines here if you want less spam.
            // ========================================================

            telemetry.addLine("Drive");
            telemetry.addData("LF", leftFrontPower);
            telemetry.addData("LB", leftBackPower);
            telemetry.addData("RF", rightFrontPower);
            telemetry.addData("RB", rightBackPower);

            telemetry.addLine("Intake");
            telemetry.addData("Intake power", intakePower);

            telemetry.addLine("Shooters");
            telemetry.addData("Shooter power", shooterPower);

            telemetry.addLine("CR Servos");
            telemetry.addData("Indexer", indexerPower);
            telemetry.addData("Servo1 (turret)", servo1Power);
            telemetry.addData("Servo2 (hood)", servo2Power);

            telemetry.update();
        }
    }
}
