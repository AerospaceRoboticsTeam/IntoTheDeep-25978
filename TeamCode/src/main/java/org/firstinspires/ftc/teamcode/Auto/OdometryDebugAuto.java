package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Libs.AR.Arm;
import org.firstinspires.ftc.teamcode.Libs.GoBilda.DriveToPoint;
import org.firstinspires.ftc.teamcode.Libs.GoBilda.GoBildaPinpointDriver;

import java.util.Locale;

@Autonomous(name="Main Left Auto", group="PinpointAuto")
@Disabled
public class OdometryDebugAuto extends LinearOpMode {

    // Initialize drive motors
    DcMotor MTR_LF;
    DcMotor MTR_RF;
    DcMotor MTR_LB;
    DcMotor MTR_RB;

    // Initialize arm
    Arm arm;

    // Declare OpMode member for the Odometry Computer
    GoBildaPinpointDriver odo;
    //OpMode member for the point-to-point navigation class
    DriveToPoint nav = new DriveToPoint(this);

    private final double power = 0.2;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        MTR_LF = hardwareMap.get(DcMotor.class, "left_front_mtr");
        MTR_RF = hardwareMap.get(DcMotor.class, "right_front_mtr");
        MTR_LB = hardwareMap.get(DcMotor.class, "left_back_mtr");
        MTR_RB = hardwareMap.get(DcMotor.class, "right_back_mtr");

        MTR_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MTR_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MTR_LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MTR_RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        MTR_LF.setDirection(DcMotorSimple.Direction.REVERSE);
        MTR_LB.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.resetPosAndIMU();

        odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
        //nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while(opModeIsActive()) {
            odo.update();


            telemetry.addData("Current Position:", odo.getPosition());

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetry.update();
        }
    }
}
