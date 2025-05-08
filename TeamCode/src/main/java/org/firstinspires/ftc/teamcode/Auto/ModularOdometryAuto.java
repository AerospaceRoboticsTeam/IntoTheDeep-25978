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

import org.firstinspires.ftc.teamcode.Libs.AR.OdometryAutoPath;

import java.util.Locale;

@Autonomous(name="Modular Auto - Beta", group="BetaAuto")
@Disabled
public class ModularOdometryAuto extends LinearOpMode {

    // Initialize drive motors
    DcMotor MTR_LF;
    DcMotor MTR_RF;
    DcMotor MTR_LB;
    DcMotor MTR_RB;

    // Initialize arm
    Arm arm;

    // Declare OpMode member for the Odometry Computer
    GoBildaPinpointDriver odo;
    // OpMode member for the point-to-point navigation class
    DriveToPoint nav = new DriveToPoint(this);

    // Targets/Points along the robot's drive path

    // Against wall, facing opposing team's side
    static final Pose2D START_POS = new Pose2D(DistanceUnit.INCH, -36, -63, AngleUnit.DEGREES, 180);
    // Behind tape, facing baskets
    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.INCH, -53, -53, AngleUnit.DEGREES, 225);
    Runnable T1_Telemetry = () -> telemetry.addLine("");
    // In front of 3rd neutral piece
    static final Pose2D TARGET_2 = new Pose2D(DistanceUnit.INCH, -36, -26, AngleUnit.DEGREES, 180);
    // Behind tape, facing baskets
    static final Pose2D TARGET_3 = new Pose2D(DistanceUnit.INCH,-53,-53, AngleUnit.DEGREES,225);
    // In front of 2nd neutral piece
    static final Pose2D TARGET_4 = new Pose2D(DistanceUnit.INCH, -46, -26, AngleUnit.DEGREES, 180);
    // Behind tape, facing baskets
    static final Pose2D TARGET_5 = new Pose2D(DistanceUnit.INCH, -53, -53, AngleUnit.DEGREES, 225);
    // In front of 1st neutral piece
    static final Pose2D TARGET_6 = new Pose2D(DistanceUnit.INCH, -56, -26, AngleUnit.DEGREES, 180);
    // Behind tape, facing baskets
    static final Pose2D TARGET_7 = new Pose2D(DistanceUnit.INCH, -53, -53, AngleUnit.DEGREES, 225);
    // In front of left side of submersible zone
    static final Pose2D TARGET_8 = new Pose2D(DistanceUnit.INCH, -28, 0, AngleUnit.DEGREES, 0);

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        MTR_LF = hardwareMap.get(DcMotor.class, "left_front_mtr");
        MTR_RF = hardwareMap.get(DcMotor.class, "right_front_mtr");
        MTR_LB = hardwareMap.get(DcMotor.class, "left_back_mtr");
        MTR_RB = hardwareMap.get(DcMotor.class, "right_back_mtr");

        // Initialize arm and claw
        arm = new Arm(this);
        arm.closeClaw();
        arm.updateClaw();

        MTR_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MTR_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MTR_LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MTR_RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MTR_LF.setDirection(DcMotorSimple.Direction.REVERSE);
        MTR_LB.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-142.0, 120.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Initializes robot's position
        odo.recalibrateIMU();
        odo.setPosition(START_POS);

        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
        //nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        // Method references
        Runnable grabSample = this::grabSample;
        Runnable scoreHighBasket = this::scoreHighBasket;

        // Constructs path
        OdometryAutoPath autoPath = new OdometryAutoPath(this, odo, MTR_LF, MTR_RF, MTR_LB, MTR_RB);
        autoPath.addWaypoint(TARGET_1, new Runnable[] {scoreHighBasket}, T1_Telemetry);

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
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
        }
    }

    public void grabSample() {
        // Moves wrist down and closes claw on sample to grab it, then moves wrist back up
        arm.setWristGrab();
        arm.updateWrist();
        arm.closeClaw();
        arm.updateClaw();
        arm.setWristGuard();
        arm.updateWrist();
    }

    public void scoreHighBasket() {
        // Moves arm up, drops piece, and resets arm to pick up a sample later
        arm.moveHighBasket();
        arm.updateSlide();
        arm.setWristDrop();
        arm.updateWrist();
        arm.openClaw();
        arm.updateClaw();
        arm.setWristGuard();
        arm.updateWrist();
        arm.moveGrab();
        arm.updateSlide();
    }
}
