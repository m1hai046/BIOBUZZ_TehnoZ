package org.firstinspires.ftc.teamcode.Biobuzz.Useful;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Globals {

    public static boolean FAILSAFE_MODE = false;

    ///poz goal aprox, de tunat
    public static double X_GOAL_RED_SUP = 59, Y_GOAL_RED_SUP = 97, X_GOAL_RED_INF = 57, Y_GOAL_RED_INF = 52;
    public static double X_GOAL_BLUE_SUP = 82, Y_GOAL_BLUE_SUP = 97, X_GOAL_BLUE_INF = 83, Y_GOAL_BLUE_INF = 52;
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
