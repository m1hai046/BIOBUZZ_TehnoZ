package org.firstinspires.ftc.teamcode.Biobuzz.Useful;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Globals {

    public static boolean FAILSAFE_MODE = false;

    ///poz goal aprox, de tunat
    public static double X_GOAL_RED_SUP = 59, Y_GOAL_RED_SUP = 92, X_GOAL_RED_INF = 60, Y_GOAL_RED_INF = 52;
    public static double X_GOAL_BLUE_SUP = 84, Y_GOAL_BLUE_SUP = 82, X_GOAL_BLUE_INF = 84, Y_GOAL_BLUE_INF = 58;

    public static double predX, predY;
    public static double goalX, goalY;
    public static double deadZone = 0.0, look_ahead_time = 0.0;/// de tunat pt sotm turela si shooter

    public static Alliance alliance;
    public static FAZE faze;

    public static void getGoal(double y) {
        if (Globals.alliance == Globals.Alliance.RED) {
            if (y > 70.7) {///sup
                goalX = Globals.X_GOAL_RED_SUP;
                goalY = Globals.Y_GOAL_RED_SUP;
            } else {///inf
                goalX = Globals.X_GOAL_RED_INF;
                goalY = Globals.Y_GOAL_RED_INF;
            }
        } else {
            if (y > 70.7) {///sup
                goalX = Globals.X_GOAL_BLUE_SUP;
                goalY = Globals.Y_GOAL_BLUE_SUP;
            } else {///inf
                goalX = Globals.X_GOAL_BLUE_INF;
                goalY = Globals.Y_GOAL_BLUE_INF;
            }
        }
    }

    public enum FAZE{
        AUTO,
        TELEOP
    }
    public enum Alliance{
        RED,
        BLUE
    }
}
