package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="CompAuto", group="Robot")
public class MainAutoMode extends LinearOpMode {

    // Declare OpMode members
    private DcMotor MTR_LF = null;
    private DcMotor MTR_LB = null;
    private DcMotor MTR_RF = null;
    private DcMotor MTR_RB = null;
    private IMU imu = null;

    @Override
    public void runOpMode() {

        // Initialize the objects and drive system variables.
        MTR_LF  = hardwareMap.get(DcMotor.class, "left_front_mtr");
        MTR_LB  = hardwareMap.get(DcMotor.class, "left_back_mtr");
        MTR_RF  = hardwareMap.get(DcMotor.class, "right_front_mtr");
        MTR_RB  = hardwareMap.get(DcMotor.class, "right_back_mtr");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        MTR_LF.setDirection(DcMotor.Direction.REVERSE);
        MTR_LB.setDirection(DcMotor.Direction.REVERSE);
        MTR_RF.setDirection(DcMotor.Direction.FORWARD);
        MTR_RB.setDirection(DcMotor.Direction.FORWARD);

        // The next two lines define Hub orientation.
        // The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        setDriveTrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveTrainZeroPower(DcMotor.ZeroPowerBehavior.FLOAT);


    }

    public void driveStraight() {

    }

    public void turnToHeading() {

    }

    public void pickUpPiece() {

    }

    public void dropPiece() {

    }

    // New class to set drive train mode for all 4 motors
    public void setDriveTrainMode(DcMotor.RunMode mode){
        MTR_LF.setMode(mode);
        MTR_LB.setMode(mode);
        MTR_RF.setMode(mode);
        MTR_RB.setMode(mode);
    }

    // New Class to set ZeroPowerBehaviour
    public void setDriveTrainZeroPower(DcMotor.ZeroPowerBehavior behaviour){
        MTR_LF.setZeroPowerBehavior(behaviour);
        MTR_LB.setZeroPowerBehavior(behaviour);
        MTR_RF.setZeroPowerBehavior(behaviour);
        MTR_RB.setZeroPowerBehavior(behaviour);
    }
}
