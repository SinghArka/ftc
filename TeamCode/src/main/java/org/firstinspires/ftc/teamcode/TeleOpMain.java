package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpMain")
public class TeleOpMain extends LinearOpMode {
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;
    private DcMotor intakeMotor;
    private DcMotorEx flywheelMotorLeft;
    private DcMotorEx flywheelMotorRight;
    private Servo ballStopper;
    private CRServo leftTransfer;
    private CRServo rightTransfer;

    @Override
    public void runOpMode() {
        // motor and servo config and naming
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back_drive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        flywheelMotorLeft = hardwareMap.get(DcMotorEx.class, "shooter_left");
        flywheelMotorRight = hardwareMap.get(DcMotorEx.class, "shooter_right");
        ballStopper = hardwareMap.get(Servo.class, "blocker_right");
        leftTransfer = hardwareMap.get(CRServo.class, "transfer_left");
        rightTransfer = hardwareMap.get(CRServo.class, "transfer_right");

        // reverse right side motors so forward joystick input drives straight
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        // reverse one flywheel motor so they don't cancel each other out and go boom
        flywheelMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // forward/backward movement
            double x = gamepad1.left_stick_x; // strafing movement
            double rx = gamepad1.right_stick_x; // left/right rotation

            /* || strafing math || strafing math || strafing math || strafing math || idk  || */
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            double leftFrontPower = (y + x + rx) / denominator;
            double leftBackPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightBackPower = (y + x - rx) / denominator;

            // flywheel ticks per second calculation || configure rpm
            double flywheelPowerRPM = 2000;
            double flywheelPowerRPS = flywheelPowerRPM / 60;
            double flywheelticks = flywheelPowerRPS * 28;

            // set power to drivetrain
            leftFrontMotor.setPower(leftFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightFrontMotor.setPower(rightFrontPower);
            rightBackMotor.setPower(rightBackPower);

            // intake and transfer
            if (gamepad1.right_trigger > 0.1) {
                intakeMotor.setPower(1.0);
                leftTransfer.setPower(-1.0);
                rightTransfer.setPower(-1.0);
            } else if (gamepad1.left_trigger > 0.1) {
                intakeMotor.setPower(-1.0);
                leftTransfer.setPower(1.0);
                rightTransfer.setPower(1.0);
            } else {
                intakeMotor.setPower(0);
                leftTransfer.setPower(0);
                rightTransfer.setPower(0);
            }

            // flywheel motor set velocity
            if (gamepad1.right_bumper) {
                flywheelMotorLeft.setVelocity(flywheelticks);
                flywheelMotorRight.setVelocity(flywheelticks);
            } else {
                flywheelMotorLeft.setVelocity(0);
                flywheelMotorRight.setVelocity(0);
            }

            // calculate motor rpm to feed telemetry || reversing the conversion from RPM to ticks per second
            double tickPerSecondFlywheelMotorLeft = (flywheelMotorLeft.getVelocity() / 28);
            double tickPerSecondFlywheelMotorRight = (flywheelMotorRight.getVelocity() / 28);

            double rpmLeftShooter = tickPerSecondFlywheelMotorLeft * 60;
            double rpmRightShooter = tickPerSecondFlywheelMotorRight * 60;

            // motor rpm and servo position telemetry
            telemetry.addData("Left Shooter Motor RPM", rpmLeftShooter);
            telemetry.addData("Right Shooter Motor RPM", rpmRightShooter);
            telemetry.addData("Ball Stopper Position", ballStopper.getPosition());
            telemetry.update();

            // ball stopper debug
            if (gamepad1.y) {
                ballStopper.setPosition(0);
            }

            if (gamepad1.x || gamepad1.b) {
                ballStopper.setPosition(0.5);
            }

            if (gamepad1.a) {
                ballStopper.setPosition(1);
            }
        }
    }
}