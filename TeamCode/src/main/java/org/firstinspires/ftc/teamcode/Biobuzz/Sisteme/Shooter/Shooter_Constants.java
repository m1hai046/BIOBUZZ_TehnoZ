package org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Shooter;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Shooter_Constants {
    public static double kP = 0.005 , kV = 0.00036, kS = 0.12;
    public static double gateOpen = 0.8, gateClosed = 0.54;
    public static double SHOOT_TOLERANCE = 100, SHOOT_TIME = 800;

    public static double FAILSAFE_VELOCITY = 1420.0, FAILSAFE_ANGLE = 0.8  , TARGET_STOPPED = 0.0;
    public static double minHoodAngle = 0.8,  maxHoodAngle = 0.8;
    public static double minShooterRpm = 1370.0, maxShooterRpm = 1530.0;

}
