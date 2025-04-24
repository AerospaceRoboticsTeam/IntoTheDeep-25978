package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Libs.AR.Arm;
import org.firstinspires.ftc.teamcode.Libs.AR.MecanumDrive;

@TeleOp(name = "25978 TeleOp", group = "TeleOp")
public class TeleOp_25978 extends LinearOpMode
{
    //@Override
    public void runOpMode()
    {
        MecanumDrive mecanumDrive;
        Arm arm;

        // Initialize the drivetrain
        mecanumDrive = new MecanumDrive(this);
        arm = new Arm (this);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive())
        {
            //**************************************************************************************
            // Drive based controls are in the MecanumDrive.java file. We should strive to have controls related to a specific
            // part of the robot contained in the library that contains the other functions of that specific part.

            // ---------------------Gamepad 1 Controls ---------------------------------------------
            // Drive control specified in Mecanum drive functions.

            // ---------------------Gamepad 2 Controls ---------------------------------------------
            // Wrist controls
            if (gamepad2.y) {
                arm.setWristGuard();
            }
            if (gamepad2.b) {
                arm.setWristDrop();
            }
            if (gamepad2.a) {
                arm.setWristGrab();
            }

            // Claw controls
            if (gamepad2.left_trigger != 0 ) {
                arm.closeClaw();
            }
            if (gamepad2.right_trigger != 0) {
                arm.openClaw();
            }

            // Linear slide height controls
            if (gamepad2.dpad_up){
                arm.moveHighBasket();
            } else if (gamepad2.dpad_left) {
                arm.moveLowBasket();
            } else if (gamepad2.dpad_down) {
                arm.moveGrab();
            }

//Experimental code below, tried by Anya 4/24
            /*
            if (gamepad2.left_stick_y >= 0.2){
                arm.manual(gamepad2.left_stick_y*2);
            }
            if (gamepad2.left_stick_y <= 0.2){
                arm.manual(gamepad2.left_stick_y*2);
            }
            */


            //**************************************************************************************
            //--------------------- Per Loop Update Code -------------------------------------------
            // Functions in this section get run every loop of the code. Anything that needs to update
            // every loop should be in here.
            mecanumDrive.drive(); // Update movement
            arm.updateWrist();    // Update the arm's position
            arm.updateClaw();     // update the claw's position
            arm.updateSlide();    // Update the arm's height.

            //**************************************************************************************
            //--------------------- TELEMETRY Code -------------------------------------------------
            // Useful telemetry data in case needed for testing and to find heading of robot
            mecanumDrive.getTelemetryData();
            arm.getTelemetryData();

            telemetry.update();
        }
    }
}