package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Libs.AR.Arm;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Libs.GoBilda.DriveToPoint;
import org.firstinspires.ftc.teamcode.Libs.GoBilda.GoBildaPinpointDriver;

import java.util.Locale;

@Autonomous(name="Pinpoint Navigation Example", group="Pinpoint")
public class SensorPinpointDriveToPoint extends LinearOpMode {

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

    enum StateMachine {
        WAITING_FOR_START,
        COMPLETED_PATH,
        DRIVE_TO_TARGET_1,
        DRIVE_TO_TARGET_2,
        DRIVE_TO_TARGET_3,
        DRIVE_TO_TARGET_4,
        DRIVE_TO_TARGET_5,
        DRIVE_TO_TARGET_6,
        DRIVE_TO_TARGET_7,
        DRIVE_TO_TARGET_8
    }

    // Targets/Points along the robot's drive path

    // Against wall, facing opposing team's side
    static final Pose2D START_POS = new Pose2D(DistanceUnit.INCH, -36, -63, AngleUnit.DEGREES, 180);
    // Behind tape, facing baskets
    static final Pose2D TARGET_1 = new Pose2D(DistanceUnit.INCH, -53, -53, AngleUnit.DEGREES, 225);
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

    private final double power = 0.7;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        MTR_LF = hardwareMap.get(DcMotor.class, "left_front_mtr");
        MTR_RF = hardwareMap.get(DcMotor.class, "right_front_mtr");
        MTR_LB = hardwareMap.get(DcMotor.class, "left_back_mtr");
        MTR_RB = hardwareMap.get(DcMotor.class, "right_back_mtr");

        arm = new Arm(this);

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

        // Initializes state machine
        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            odo.update();

            switch (stateMachine){
                case WAITING_FOR_START:
                    stateMachine = StateMachine.DRIVE_TO_TARGET_1;
                    break;
                case DRIVE_TO_TARGET_1:
                    if (nav.driveTo(odo.getPosition(), TARGET_1, power, 0)){
                        // TODO: Move arm to score
                        telemetry.addLine("In front of baskets, attempting to score");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_2;
                    }
                    break;
                case DRIVE_TO_TARGET_2:
                    if (nav.driveTo(odo.getPosition(), TARGET_2, power, 0)){
                        // TODO: Move arm to pick up piece
                        telemetry.addLine("Behind 3rd neutral sample");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_3;
                    }
                    break;
                case DRIVE_TO_TARGET_3:
                    if(nav.driveTo(odo.getPosition(), TARGET_3, power, 0)){
                        // TODO: Move arm to score
                        telemetry.addLine("In front of baskets, attempting to score");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_4;
                    }
                    break;
                case DRIVE_TO_TARGET_4:
                    if(nav.driveTo(odo.getPosition(), TARGET_4, power,0)){
                        // TODO: Move arm to pick up piece
                        telemetry.addLine("Behind 2nd neutral sample");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_5;
                    }
                    break;
                case DRIVE_TO_TARGET_5:
                    if(nav.driveTo(odo.getPosition(), TARGET_5, power,0)){
                        // TODO: Move arm to score
                        telemetry.addLine("In front of baskets, attempting to score");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_6;
                    }
                    break;
                case DRIVE_TO_TARGET_6:
                    if(nav.driveTo(odo.getPosition(), TARGET_6, power,0)){
                        // TODO: Move arm to pick up piece
                        telemetry.addLine("Behind 1st neutral sample");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_7;
                    }
                    break;
                case DRIVE_TO_TARGET_7:
                    // TODO: Move arm to score
                    if(nav.driveTo(odo.getPosition(), TARGET_7, power,0)){
                        telemetry.addLine("In front of baskets, attempting to score");
                        stateMachine = StateMachine.DRIVE_TO_TARGET_8;
                    }
                    break;
                case DRIVE_TO_TARGET_8:
                    if(nav.driveTo(odo.getPosition(), TARGET_8, power,0)){
                        // TODO: Reset arm for TeleOP, or move it into submersible zone
                        telemetry.addLine("Next to submersible zone, path completed");
                        stateMachine = StateMachine.COMPLETED_PATH;
                    }
                    break;
            }

            //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
            MTR_LF.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
            MTR_RF.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
            MTR_LB.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
            MTR_RB.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));

            telemetry.addData("Current state:", stateMachine);

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetry.update();
        }
    }
}
