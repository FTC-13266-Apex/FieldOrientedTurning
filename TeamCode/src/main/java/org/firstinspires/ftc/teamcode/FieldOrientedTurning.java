package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.HashMap;
import java.util.Map;

@TeleOp
@Config
public class FieldOrientedTurning extends LinearOpMode {
    public static double STRAFE_WEIGHT = 1.1;
    public static double MIN_HEADING_P = 0.05;
    public static double MAX_HEADING_P = 1.0;
    public static double STICK_THRESHOLD = 0.05;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftRear = hardwareMap.dcMotor.get("leftRear");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightRear = hardwareMap.dcMotor.get("rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                imu.resetYaw();
            }

            double leftY = -gamepad1.left_stick_y; // Remember, this is reversed!
            double leftX = gamepad1.left_stick_x * STRAFE_WEIGHT; // Counteract imperfect strafing
            double rightY = -gamepad1.right_stick_y;
            double rightX = -gamepad1.right_stick_x;

            double botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            Vector2d rotated2d = new Vector2d(leftX, leftY).rotated(botHeading);

            double x = rotated2d.getX();
            double y = rotated2d.getY();

            Vector2d turn2d = new Vector2d(rightX, rightY);
            double stickHeading = turn2d
                    .rotated(Math.PI / 2) // Rotate by Pi / 2 Radians (90 degrees)
                    .angle() - Math.PI; // Subtract Pi radians (180 degrees) from final angle

            // Find the lowest of the 3 coterminal angles
            double defaultHeadingError = (botHeading - stickHeading); // Default angle
            double lowHeadingError = defaultHeadingError - (Math.PI * 2); // Low coterminal angle (360 degrees off from original)
            double highHeadingError = defaultHeadingError + (Math.PI * 2); // High coterminal angle (360 degrees off from original)

            // Create map of the absolute value of each value to the original value
            Map<Double, Double> errorMap = new HashMap<>();
            errorMap.put(Math.abs(defaultHeadingError), defaultHeadingError);
            errorMap.put(Math.abs(lowHeadingError), lowHeadingError);
            errorMap.put(Math.abs(highHeadingError), highHeadingError);

            // Find the lowest absolute value key and get the value corresponding to that key
            double headingError = errorMap.entrySet().stream()
                    .min(Map.Entry.comparingByKey()) // Find the lowest entry by finding the key with the lowest value
                    .get().getValue(); // Get the value for this entry

            double distance = turn2d.distTo(new Vector2d(0,0));


            double headingP = MIN_HEADING_P + ((MAX_HEADING_P - MIN_HEADING_P) * distance);
            double turn = headingError * -headingP;

            if (distance < STICK_THRESHOLD) turn = 0;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);
            double frontLeftPower = (y + x + turn) / denominator;
            double backLeftPower = (y - x + turn) / denominator;
            double frontRightPower = (y - x - turn) / denominator;
            double backRightPower = (y + x - turn) / denominator;

            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);

            telemetry.addData("heading error", Math.toDegrees(botHeading - stickHeading));
            telemetry.addData("heading", Math.toDegrees(botHeading));
            telemetry.addData("theta", Math.toDegrees(stickHeading));

            telemetry.addData("low heading error", lowHeadingError);
            telemetry.addData("default heading error", defaultHeadingError);
            telemetry.addData("high heading error", highHeadingError);

            telemetry.update();
        }
    }

    public double min(double... nums) {
        double min = nums[0];
        for (double num: nums) {
            min = Math.min(num, min);
        }
        return min;
    }
}
