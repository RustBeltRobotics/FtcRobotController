package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.RobotLog;

public final class DebugLog {

    private DebugLog() { }

    public static void log(String format, Object... args) {
        RobotLog.dd("RBR", format, args);
    }
}
