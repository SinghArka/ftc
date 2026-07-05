package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Arka's Simple Mecanum Drive")
public class SimpleMecanumDrive extends LinearOpMode {
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;
    private DcMotor intakeMotor;
    private Servo ballStopper;

    @Override
    public void runOpMode() {
        // motor config and naming
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back_drive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        ballStopper = hardwareMap.get(Servo.class, "blocker_right");

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // forward/backward movement
            double x = gamepad1.left_stick_x; // strafing movement plsssswork
            double rx = gamepad1.right_stick_x; // left/right rotation

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            double leftFrontPower = (y + x + rx) / denominator;
            double leftBackPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightBackPower = (y + x - rx) / denominator;

            leftFrontMotor.setPower(leftFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightFrontMotor.setPower(rightFrontPower);
            rightBackMotor.setPower(rightBackPower);

            if (gamepad1.right_trigger > 0.1) {
                intakeMotor.setPower(1.0);
            } else if (gamepad1.left_trigger > 0.1) {
                intakeMotor.setPower(-1.0);
            } else {
                intakeMotor.setPower(0);
            }

            if (gamepad1.y) {
                ballStopper.setPosition(0);
            }

            if (gamepad1.x || gamepad1.b) {
                ballStopper.setPosition(0.5);
            }

            if (gamepad1.a) {
                ballStopper.setPosition(1);
            }

            // telemetry data
            telemetry.addData("Ball Stopper Position", ballStopper.getPosition());
            telemetry.update();
        }
    }
}
