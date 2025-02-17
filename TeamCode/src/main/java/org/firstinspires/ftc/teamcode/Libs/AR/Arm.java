package org.firstinspires.ftc.teamcode.Libs.AR;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm
{
    private DcMotor MTR_VS;
    private int slideGrab = 50;
    private int slideLow = 2200;
    private int slideHigh = 4100;
    private int currentSlidePosition = 50;
    private boolean slideChangingState = false;
    private static final double maxSlidePower = 0.7;

    private Servo SRV_CLAW;
    private double clawOpen = 0.0;
    private double clawClosed = 0.20;
    private double currentClawPosition;

    private Servo SRV_WRIST;
    private double wristGuard = 0.0;
    private double wristDrop = 0.11;
    private double wristGrab = 0.21;
    private double currentWristPosition;

    LinearOpMode bot;

    public Arm( LinearOpMode iBot )
    {
        bot = iBot;

        MTR_VS = bot.hardwareMap.get(DcMotor.class, "viper_mtr");
        MTR_VS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Set Motor to 0 ticks.
        MTR_VS.setDirection(DcMotor.Direction.REVERSE);
        MTR_VS.setTargetPosition(currentSlidePosition);
        MTR_VS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MTR_VS.setPower(maxSlidePower);
        MTR_VS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SRV_CLAW = bot.hardwareMap.get(Servo.class, "claw");

        SRV_WRIST = bot.hardwareMap.get(Servo.class, "wrist");
        SRV_WRIST.setDirection(Servo.Direction.REVERSE);
    }
    public void openClaw() {
        currentClawPosition = clawOpen;
    }

    public void closeClaw() {
        currentClawPosition = clawClosed;
    }

    public void updateClaw() {
        SRV_CLAW.setPosition(currentClawPosition);
    }

    public void setWristGuard() {
        currentWristPosition = wristGuard;
    }

    public void setWristDrop() {
        currentWristPosition = wristDrop;
    }

    public void setWristGrab() {
        currentWristPosition = wristGrab;
    }

    public void updateWrist() {
        SRV_WRIST.setPosition(currentWristPosition);
    }

    public void moveGrab() {
        bot.telemetry.addData("Status", "In moveGrab" );

        currentSlidePosition = slideGrab;
        slideChangingState = true;
    }

    public void moveLowBasket() {
        bot.telemetry.addData("Status", "In moveLowBasket" );

        currentSlidePosition = slideLow;
        slideChangingState = true;
    }

    public void moveHighBasket() {
        bot.telemetry.addData("Status", "In moveHighBasket" );

        currentSlidePosition = slideHigh;
        slideChangingState = true;
   }

    public void updateSlide()
    {
        bot.telemetry.addData("Status: ", "In update slide" );

        if( slideChangingState ) {
            MTR_VS.setPower(maxSlidePower);
            slideChangingState = false;
        }

        // See if the slide
        if ( !slideIsMoving() &&
              ( ( MTR_VS.getCurrentPosition() <= slideGrab + 5 ) &&
                ( MTR_VS.getCurrentPosition() >= slideGrab - 5 ) ) ){
            MTR_VS.setPower(0.0);
        }

        MTR_VS.setTargetPosition(currentSlidePosition);
    }

    public boolean slideIsMoving() {
        return MTR_VS.isBusy();
    }

    public boolean wristIsMoving() {
        if(SRV_WRIST.getDirection() == Servo.Direction.FORWARD) {
            return SRV_WRIST.getPosition() < currentWristPosition;
        }
        else if(SRV_WRIST.getDirection() == Servo.Direction.REVERSE) {
            return SRV_WRIST.getPosition() > currentWristPosition;
        }
        else {
            return false;
        }
    }

    public boolean clawIsMoving() {
        if(SRV_CLAW.getDirection() == Servo.Direction.FORWARD) {
            return SRV_CLAW.getPosition() < currentClawPosition;
        }
        else if(SRV_CLAW.getDirection() == Servo.Direction.REVERSE) {
            return SRV_CLAW.getPosition() > currentClawPosition;
        }
        else {
            return false;
        }
    }

    public void getTelemetryData()
    {
        bot.telemetry.addData("Wrist Current Position: ", SRV_WRIST.getPosition());
        bot.telemetry.addData(" - Desired Wrist Position: ", currentWristPosition );
        bot.telemetry.addData("Claw Current Position: ", SRV_CLAW.getPosition());
        bot.telemetry.addData(" - Desired Claw Position: ", currentClawPosition );
        bot.telemetry.addData("Slide Current Position: ", MTR_VS.getCurrentPosition());
        bot.telemetry.addData(" - Desired Slide Position: ", currentSlidePosition);
        bot.telemetry.addData(" - Current Slide Power: ", MTR_VS.getPower());
    }
}