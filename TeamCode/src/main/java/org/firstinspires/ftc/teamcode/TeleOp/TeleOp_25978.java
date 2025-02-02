package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Libs.AR.Arm;
import org.firstinspires.ftc.teamcode.Libs.AR.MecanumDrive;

@TeleOp(name = "25978 TeleOp", group = "TeleOp")
public class TeleOp_25978 extends LinearOpMode
{
    private MecanumDrive mecanumDrive;

    private Arm slide;

    //@Override
    public void runOpMode()
    {
        // Initialize the drivetrain
        mecanumDrive = new MecanumDrive(this);
        slide = new Arm (this);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive())
        {
            //**************************************************************************************
            // Drive based controls are in the Mecanum.java file. We should strive to have controls related to a specific
            // part of the robot contained in the library that contains the other functions of that specific part.

            // ---------------------Gamepad 1 Controls ---------------------------------------------
            // Drive control specified in Mecanum drive functions.

            // ---------------------Gamepad 2 Controls ---------------------------------------------
            // Wrist controls
            if (gamepad2.x) {
                slide.setWristGuard();
            }
            if (gamepad2.y) {
                slide.setWristDrop();
            }
            if (gamepad2.b) {
                slide.setWristGrab();
            }

            // Claw controls
            if (gamepad2.left_trigger != 0) {
                slide.openClaw();
            }
            if (gamepad2.right_trigger != 0 ) {
                slide.closeClaw();
            }

            // Linear slide controls
            if (gamepad2.dpad_up){
                slide.moveHighBasket();
            } else if (gamepad2.dpad_left) {
                slide.moveLowBasket();
            } else if (gamepad2.dpad_down) {
                slide.moveGrab();
            }

            //**************************************************************************************
            //--------------------- Per Loop Update Code -------------------------------------------
            // Functions in this section get run every loop of the code. Anything that needs to update
            // every loop should be in here.
            mecanumDrive.drive(); // Update movement
            slide.updateWrist();  // Update the arm's position
            slide.updateClaw();   // update the claw's position

            //**************************************************************************************
            //--------------------- TELEMETRY Code -------------------------------------------------
            // Useful telemetry data in case needed for testing and to find heading of robot
            mecanumDrive.getTelemetryData();
            slide.getTelemetryData();
            telemetry.update();
        }
    }
}