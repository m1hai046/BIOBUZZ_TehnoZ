package org.firstinspires.ftc.teamcode.Biobuzz.Useful;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Globals {

    public static boolean FAILSAFE_MODE = false;

    ///poz goal aprox, de tunat
    public static double X_GOAL_RED_SUP = 58, Y_GOAL_RED_SUP = 82, X_GOAL_RED_INF = 63, Y_GOAL_RED_INF = 58;
    public static double X_GOAL_BLUE_SUP = 84, Y_GOAL_BLUE_SUP = 82, X_GOAL_BLUE_INF = 84, Y_GOAL_BLUE_INF = 58;

    public static double predX, predY;
    public static double deadZone = 0.0, look_ahead_time = 0.0;/// de tunat pt sotm turela si shooter

    public static Alliance alliance;
    public static FAZE faze;

    public enum FAZE{
        AUTO,
        TELEOP
    }
    public enum Alliance{
        RED,
        BLUE
    }
}
