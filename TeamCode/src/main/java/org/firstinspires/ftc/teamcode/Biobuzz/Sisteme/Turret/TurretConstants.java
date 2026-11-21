package org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Turret;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class TurretConstants {
    public static double GOAL_DEPTH = 12.04;
    public static double SHOOTER_AIM_GAIN = 1.0, TURRET_AIM_FRACTION = 1;
    public static double MAX_AIM_SHIFT = 8.0;
    public static double MIN_ANGLE = -170.0, MAX_ANGLE = 180.0, MIN_POS = 0.07, MAX_POS = 0.96;///de tunat
    public static double shooterOffset = -1.811; ///de pus offset ul de la poz shooter ului
    public static double failsafe_position = 0.5025;
    public static double centerOffset = -3.0, cOffset, offset = 0.0;;


}
