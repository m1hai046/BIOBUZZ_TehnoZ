package org.firstinspires.ftc.teamcode.Biobuzz.Useful;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Globals {

    public static boolean FAILSAFE_MODE = false;

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
