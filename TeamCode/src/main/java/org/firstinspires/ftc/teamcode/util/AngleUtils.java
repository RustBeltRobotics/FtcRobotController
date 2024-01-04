package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.model.TurnDirection;

public final class AngleUtils {

    private AngleUtils() { }

    public static final double getRelativeTurnAngle(double angle, TurnDirection turnDirection) {
        return TurnDirection.LEFT.equals(turnDirection) ? angle : -angle;
    }
}
