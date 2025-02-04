package org.firstinspires.ftc.teamcode.Libs.AR;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TeleOp.TeleOp_25978;

public class Arm {
    private DcMotor MTR_VS;
    private Servo SRV_WRIST;
    private Servo SRV_CLAW;

    private static final double MTR_VS_PW = 0.7;

    private double dClawOpen = 0.0;
    private double dClawClosed = 0.20;
    private double dCurrentClawPosition;

    private double dWristStart  = 0.0;
    private double dWristGuard = 0.0;
    private double dWristDrop = 0.11;
    private double dWristGrab = 0.22;
    private double dCurrentWristPosition;

    private int dCurrentHeightPosition;

    LinearOpMode bot;

    public Arm( LinearOpMode iBot )
    {
        bot = iBot;

        MTR_VS = bot.hardwareMap.get(DcMotor.class, "viper_mtr");
        MTR_VS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Set Motor to 0 ticks.
        MTR_VS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MTR_VS.setDirection(DcMotor.Direction.REVERSE);

        SRV_CLAW = bot.hardwareMap.get(Servo.class, "claw");

        SRV_WRIST = bot.hardwareMap.get(Servo.class, "wrist");
        SRV_WRIST.setDirection(Servo.Direction.REVERSE);
    }
    public void openClaw() {
        dCurrentClawPosition = dClawOpen;
    }

    public void closeClaw() {
        dCurrentClawPosition = dClawClosed;
    }

    public void updateClaw() {
        SRV_CLAW.setPosition( dCurrentClawPosition );
    }

    public void setWristStart() {
        dCurrentWristPosition = dWristStart;
    }

    public void setWristGuard() {
        dCurrentWristPosition = dWristGuard;
    }

    public void setWristDrop() {
        dCurrentWristPosition = dWristDrop;
    }

    public void setWristGrab() {
        dCurrentWristPosition = dWristGrab;
    }

    public void updateWrist() {
        SRV_WRIST.setPosition(dCurrentWristPosition);
    }

    public void moveGrab() {
        dCurrentHeightPosition = 20;  // Todo: All heights need to be adjusted.
    }

    public void moveLowBasket() {
        dCurrentHeightPosition = 1400;  // Todo: All heights need to be adjusted.
    }

    public void moveHighBasket() {
        dCurrentHeightPosition = 3000;  // Todo: All heights need to be adjusted.
   }

    public void updateHeight() {
        MTR_VS.setTargetPosition(dCurrentHeightPosition);;
    }

    public void getTelemetryData() {
        bot.telemetry.addData("Wrist Position: ", SRV_WRIST.getPosition());
        bot.telemetry.addData("Claw Position: ", SRV_CLAW.getPosition());
        bot.telemetry.addData("MTR_LVS Position: ", MTR_VS.getCurrentPosition());
        bot.telemetry.addData("Wrist Current Position: ", dCurrentWristPosition );
        bot.telemetry.addData("Claw Current Position: ", dCurrentClawPosition );
    }
}