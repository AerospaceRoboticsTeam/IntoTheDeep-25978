package org.firstinspires.ftc.teamcode.Libs.AR;

import org.firstinspires.ftc.teamcode.Libs.AR.Waypoint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.Libs.GoBilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Libs.GoBilda.DriveToPoint;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.function.*;

public class OdometryAutoPath {
    public ArrayList<Waypoint> path = new ArrayList<>();
    private static final double power = 0.2;
    private final LinearOpMode bot;
    private final DriveToPoint nav;
    private final GoBildaPinpointDriver odo;

    private final DcMotor leftFrontMotor;
    private final DcMotor rightFrontMotor;
    private final DcMotor leftBackMotor;
    private final DcMotor rightBackMotor;

    public OdometryAutoPath(LinearOpMode opMode, GoBildaPinpointDriver pinpointDriver, DcMotor leftFrontMotor, DcMotor rightFrontMotor, DcMotor leftBackMotor, DcMotor rightBackMotor) {
        bot = opMode;
        nav = new DriveToPoint(bot);
        odo = pinpointDriver;

        this.leftFrontMotor = leftFrontMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.leftBackMotor = leftBackMotor;
        this.rightBackMotor = rightBackMotor;
    }

    public void addTarget(Pose2D target, Consumer<Void>[] functions, Consumer<Void> telemetry) {
        Waypoint newWaypoint = new Waypoint(target, functions, telemetry);
        path.add(newWaypoint);
    }

    public void start() {
        for(int i = 0; i < path.size() && bot.opModeIsActive(); i++) {
            odo.update();
            Waypoint currentWaypoint = path.get(i);

            while(nav.driveTo(odo.getPosition(), currentWaypoint.target, power, 0) && bot.opModeIsActive()) {
                odo.update();

                leftFrontMotor.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
                rightFrontMotor.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
                leftBackMotor.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
                rightBackMotor.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
            }

            for(Consumer<Void> function : currentWaypoint.functions) {
                function.accept(null);
            }

            currentWaypoint.telemetry.accept(null);
            bot.telemetry.update();
        }
    }
}
