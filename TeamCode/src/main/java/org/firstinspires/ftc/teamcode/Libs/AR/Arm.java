package org.firstinspires.ftc.teamcode.Libs.AR;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_25978;

public class Arm
{
    private DcMotor MTR_VS;
    private int currentSlidePosition = 10;
    private static final double currentSlidePower = 0.7;
    private int slideGrab = 10;
    private int slideLow = 2200;
    private int slideHigh = 4000;

    private Servo SRV_CLAW;
    private double clawOpen = 0.0;
    private double clawClosed = 0.20;
    private double currentClawPosition;

    private Servo SRV_WRIST;
    private double wristGuard = 0.0;
    private double wristDrop = 0.11;
    private double wristGrab = 0.22;
    private double currentWristPosition;

    TeleOp_25978 bot;

    public Arm( TeleOp_25978 iBot )
    {
        bot = iBot;

        MTR_VS = bot.hardwareMap.get(DcMotor.class, "viper_mtr");
        MTR_VS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Set Motor to 0 ticks.
        MTR_VS.setDirection(DcMotor.Direction.REVERSE);
        MTR_VS.setPower(currentSlidePower);
        MTR_VS.setTargetPosition(currentSlidePosition);
        MTR_VS.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        SRV_CLAW.setPosition( currentClawPosition );
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
        SRV_WRIST.setPosition( currentWristPosition);
    }

    public void moveGrab() {
        currentSlidePosition = slideGrab;
        bot.telemetry.addData("Status", "In moveGrab" );
    }

    public void moveLowBasket() {
        currentSlidePosition = slideLow;
        bot.telemetry.addData("Status", "In moveLowBasket" );
    }

    public void moveHighBasket() {
        currentSlidePosition = slideHigh;
        bot.telemetry.addData("Status", "In moveHighBasket" );
   }

    public void updateSlide() {
        MTR_VS.setTargetPosition( currentSlidePosition);;
    }

    public void getTelemetryData() {
        bot.telemetry.addData("Wrist Current Position: ", SRV_WRIST.getPosition());
        bot.telemetry.addData("- Desired Wrist Position: ", currentWristPosition );
        bot.telemetry.addData("Claw Current Position: ", SRV_CLAW.getPosition());
        bot.telemetry.addData("- Desired Claw Position: ", currentClawPosition );
        bot.telemetry.addData("Slide Current Position: ", MTR_VS.getCurrentPosition());
        bot.telemetry.addData("- Desired Slide Position: ", currentSlidePosition);
        bot.telemetry.addData("- Slide Power: ", currentSlidePower);
    }
}