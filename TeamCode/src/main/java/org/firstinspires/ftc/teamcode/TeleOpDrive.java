package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp", group = "Drive")
public class TeleOpDrive extends LinearOpMode {

    // Drivetrain motors
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    @Override
    public void runOpMode() {

        // 1) Map motors to configuration names
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        // 2) Set directions so "forward" on the sticks = robot moves forward
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // 3) Basic motor settings
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("TeleOp ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 4) Read sticks (G1)
            double y  = -gamepad1.left_stick_y;   // forward / back
            double x  =  gamepad1.left_stick_x;   // strafe
            double rx =  gamepad1.right_stick_x;  // turn

            // Deadzone to avoid drift
            if (Math.abs(y)  < 0.05) y  = 0;
            if (Math.abs(x)  < 0.05) x  = 0;
            if (Math.abs(rx) < 0.05) rx = 0;

            // 5) Mecanum power calculation
            double leftFrontPower  = y + x + rx;
            double leftBackPower   = y - x + rx;
            double rightFrontPower = y - x - rx;
            double rightBackPower  = y + x - rx;

            // Normalize so no value is > 1.0
            double max = Math.max(1.0,
                    Math.max(Math.abs(leftFrontPower),
                            Math.max(Math.abs(leftBackPower),
                                    Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower)))));

            leftFrontPower  /= max;
            leftBackPower   /= max;
            rightFrontPower /= max;
            rightBackPower  /= max;

            // 6) Optional overall speed scale (change this number if too fast/slow)
            double speedScale = 0.7;
            leftFrontPower  *= speedScale;
            leftBackPower   *= speedScale;
            rightFrontPower *= speedScale;
            rightBackPower  *= speedScale;

            // 7) Send power to motors
            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);

            // 8) Telemetry
            telemetry.addData("LF", leftFrontPower);
            telemetry.addData("LB", leftBackPower);
            telemetry.addData("RF", rightFrontPower);
            telemetry.addData("RB", rightBackPower);
            telemetry.update();
        }
    }
}

