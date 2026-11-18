package org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Turret;

import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Shooter.Shooter_Constants;
import org.firstinspires.ftc.teamcode.Biobuzz.Useful.Globals;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class Turret {

    public static double target_angle, relative_angle, target_position;
    public static double goalX, goalY;
    public static double shooterWorldX, shooterWorldY;



    public static double approachDeg, aimShift, aimX, aimY;
    public CachingServo servo_left, servo_right;


    public Turret(HardwareMap hardwareMap){
        servo_left = new CachingServo(hardwareMap.get(Servo.class, "Turela_left"));
        servo_right = new CachingServo(hardwareMap.get(Servo.class, "Turela_right"));
        TurretConstants.offset = 0;
    }

    public void update_turret(double x, double y, double heading){

        shooterWorldX = x + (TurretConstants.shooterOffset * Math.cos(heading));
        shooterWorldY = y + (TurretConstants.shooterOffset * Math.sin(heading));


        if(Globals.alliance == Globals.Alliance.RED){
            if (y >70.7) {///sup
                goalX = Globals.X_GOAL_RED_SUP;
                goalY = Globals.Y_GOAL_RED_SUP;
                TurretConstants.cOffset = -TurretConstants.centerOffset;
            }
            else{///inf
                goalX = Globals.X_GOAL_RED_INF;
                goalY = Globals.Y_GOAL_RED_INF;
                TurretConstants.cOffset = TurretConstants.centerOffset;
            }
        }

        else {
            if (y >70.7) {///sup
                goalX = Globals.X_GOAL_BLUE_SUP;
                goalY = Globals.Y_GOAL_BLUE_SUP;
                TurretConstants.centerOffset = -TurretConstants.centerOffset;
            }
            else{///inf
                goalX = Globals.X_GOAL_BLUE_INF;
                goalY = Globals.Y_GOAL_BLUE_INF;
                TurretConstants.cOffset = TurretConstants.centerOffset;
            }
        }
        aimPointOffset(shooterWorldX, shooterWorldY);


        target_angle = Math.atan2(aimY - shooterWorldY, aimX - shooterWorldX);
        target_angle = AngleUnit.normalizeRadians(target_angle);
        heading = AngleUnit.normalizeRadians(heading);

        relative_angle = Math.toDegrees(target_angle - heading);
        relative_angle = AngleUnit.normalizeDegrees(relative_angle) + TurretConstants.offset + TurretConstants.cOffset;
        relative_angle = Math.max(TurretConstants.MIN_ANGLE, Math.min(TurretConstants.MAX_ANGLE, relative_angle));
        target_position = Range.scale(relative_angle, TurretConstants.MIN_ANGLE, TurretConstants.MAX_ANGLE, TurretConstants.MIN_POS, TurretConstants.MAX_POS);

        if(!Globals.FAILSAFE_MODE){
            moveTo(target_position);
        }
        else{
            moveTo(TurretConstants.failsafe_position);
        }
    }

    public void update_sotm(double y, double heading){

        double predX = Globals.predX;
        double predY = Globals.predY;

        shooterWorldX = predX + (TurretConstants.shooterOffset * Math.cos(heading));
        shooterWorldY = predY + (TurretConstants.shooterOffset * Math.sin(heading));


        if(Globals.alliance == Globals.Alliance.RED){
            if (y >70.7) {///sup
                goalX = Globals.X_GOAL_RED_SUP;
                goalY = Globals.Y_GOAL_RED_SUP;
            }
            else{///inf
                goalX = Globals.X_GOAL_RED_INF;
                goalY = Globals.Y_GOAL_RED_INF;
            }
        }

        else {
            if (y >70.7) {///sup
                goalX = Globals.X_GOAL_BLUE_SUP;
                goalY = Globals.Y_GOAL_BLUE_SUP;
            }
            else{///inf
                goalX = Globals.X_GOAL_BLUE_INF;
                goalY = Globals.Y_GOAL_BLUE_INF;
            }
        }


        aimPointOffset(shooterWorldX, shooterWorldY);

        target_angle = Math.atan2(aimY - shooterWorldY, aimX - shooterWorldX);
        target_angle = AngleUnit.normalizeRadians(target_angle);
        heading = AngleUnit.normalizeRadians(heading);

        relative_angle = Math.toDegrees(AngleUnit.normalizeRadians(target_angle - heading)) + TurretConstants.offset + TurretConstants.centerOffset;
//        relative_angle = AngleUnit.normalizeDegrees(relative_angle + 180);   //pt turela pe invers
        ///am comentat o ca nu mai avem turela la spate
        relative_angle = Math.max(TurretConstants.MIN_ANGLE, Math.min(TurretConstants.MAX_ANGLE, relative_angle));

        target_position = Range.scale(relative_angle, TurretConstants.MIN_ANGLE, TurretConstants.MAX_ANGLE, TurretConstants.MIN_POS, TurretConstants.MAX_POS);

        if (!Globals.FAILSAFE_MODE)
            moveTo(target_position);
        else
            moveTo(TurretConstants.failsafe_position);
    }

    public void moveTo(double x){
        servo_left.setPosition(x);
        servo_right.setPosition(x);
    }

    private static void aimPointOffset(double sx, double sy){///  sx-shooterX   sy-shooterY
        double lateral = sx - goalX;
        double forward = Math.max(Math.abs(sy - goalY), 1.0);

        approachDeg = Math.toDegrees(Math.atan2(lateral, forward));

        aimShift = Utils.clamp(TurretConstants.SHOOTER_AIM_GAIN * (TurretConstants.GOAL_DEPTH / 2.0) * lateral / forward,
                -TurretConstants.MAX_AIM_SHIFT, TurretConstants.MAX_AIM_SHIFT);

        aimX = goalX + aimShift * TurretConstants.TURRET_AIM_FRACTION;

        aimY = goalY;
    }

}
