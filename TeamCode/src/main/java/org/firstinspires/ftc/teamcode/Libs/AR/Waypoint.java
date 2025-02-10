package org.firstinspires.ftc.teamcode.Libs.AR;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.function.*;

public class Waypoint {
    Pose2D target;
    Consumer<Void>[] functions;
    Consumer<Void> telemetry;

    public Waypoint(Pose2D target, Consumer<Void>[] functions, Consumer<Void> telemetry) {
        this.target = target;
        this.functions = functions;
        this.telemetry = telemetry;
    }
}
