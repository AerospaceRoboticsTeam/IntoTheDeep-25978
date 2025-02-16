package org.firstinspires.ftc.teamcode.Libs.AR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Libs.GoBilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_25978;

public class MecanumDrive
{
    private double y; //value of y on joystick
    private double x; //value of x on joystick
    private double rx; //rotation value
    private double rotX; //rotational value of x
    private double rotY; //rotational value of y
    private double botHeading; //direction of bot

    //Values of power
    private double leftFrontPower;
    private double leftBackPower;
    private double rightFrontPower;
    private double rightBackPower;
    private static double xSensitivity = 1.3;
    private static double ySensitivity = 0.75;
    private static double rxSensitivity = 0.75;

    private double fullPower = 1.0;
    private double cutPower = 0.5;

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Declare OpMode member for the Odometry Computer
    GoBildaPinpointDriver odo;
    LinearOpMode bot;
    TeleOp_25978 secondbot;

    public MecanumDrive(LinearOpMode iBot)
    {
        bot = iBot;
        frontLeft = iBot.hardwareMap.dcMotor.get("left_front_mtr");
        frontRight = iBot.hardwareMap.dcMotor.get("right_front_mtr");
        backLeft = iBot.hardwareMap.dcMotor.get("left_back_mtr");
        backRight = iBot.hardwareMap.dcMotor.get("right_back_mtr");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Initialize Gobilda Pinpoint Computer
        odo = iBot.hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-142.0, 120.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        stop(); // Set all the drive wheels to stop.
    }

    public void drive()
    {
        //**************************************************************************************
        //---------------------Gamepad 1 Controls/Drivetrain Movement----------------------//
        y = -(bot.gamepad1.left_stick_y) * ySensitivity; // Reversed Value
        x = bot.gamepad1.left_stick_x * xSensitivity ; // The double value on the left is a sensitivity setting (change when needed)
        rx = bot.gamepad1.right_stick_x * rxSensitivity; // Rotational Value

        // Todo: Check this
        // Find the first angle (Yaw) to get the robot heading.
        botHeading = odo.getHeading();

        // Translate to robot heading from field heading for motor values
        rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // The Robot drive normally with nothing pressed but if you need to slow everything down, hit the left trigger on gamepad 1.
        if (bot.gamepad1.left_trigger != 0) {
            setPower(fullPower);
        }
        else {
            setPower(cutPower);
        }

        // Denominator is the largest motor power
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        leftFrontPower = (rotY + rotX + rx) / denominator;
        leftBackPower = (rotY - rotX + rx) / denominator;
        rightFrontPower = (rotY - rotX - rx) / denominator;
        rightBackPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(leftFrontPower * cutPower);
        backLeft.setPower(leftBackPower * cutPower);
        frontRight.setPower(rightFrontPower * cutPower);
        backRight.setPower(rightBackPower * cutPower);
    }

    public void stop()
    {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void setPower(double power)
    {
        cutPower = power;
    }

    // Returns power to left front motor
    public double getLeftFrontPower()
    {
        return leftFrontPower;
    }

    // Returns power to left back motor
    public double getLeftBackPower()
    {
        return leftBackPower;
    }

    // Returns power to right front motor
    public double getRightFrontPower()
    {
        return rightFrontPower;
    }

    // Returns power to right back motor
    public double getRightBackPower()
    {
        return rightBackPower;
    }

    public double getBotHeading()
    {
        return botHeading;
    }

    public void getTelemetryData()
    {
        bot.telemetry.addData("Left Front: ", getLeftFrontPower());
        bot.telemetry.addData("Left Back: ", getLeftBackPower());
        bot.telemetry.addData("Right Front: ", getRightFrontPower());
        bot.telemetry.addData("Right Back: ", getRightBackPower());
        bot.telemetry.addData("Heading: ", ((int) Math.toDegrees(getBotHeading())) + " degrees");
        bot.telemetry.addData("Heading: ", getBotHeading());
    }
}