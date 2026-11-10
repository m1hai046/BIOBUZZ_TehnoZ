package org.firstinspires.ftc.teamcode.Biobuzz.Sisteme;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Biobuzz.Useful.Globals;
import org.firstinspires.ftc.teamcode.Biobuzz.Useful.TeleOp_Methods;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;



///de adaugat compesare de rotatie pt turela
@Configurable
public class Turret {

    public static double target_angle, relative_angle, target_position, failsafe_position = 0.5;
    public static double shooterWorldX, shooterWorldY, shooterOffset = -1.811; ///de pus offset ul de la poz shooter ului
    public static double MIN_ANGLE = -180.0, MAX_ANGLE = 180.0, MIN_POS = 0.045, MAX_POS = 0.96;///de tunat
    public static double goalX, goalY;
    public static double offset = 0.0;

    public CachingServo servo_left, servo_right;

    public Turret(HardwareMap hardwareMap){
        servo_left = new CachingServo(hardwareMap.get(Servo.class, "Turela_left"));
        servo_right = new CachingServo(hardwareMap.get(Servo.class, "Turela_right"));
    }

    public void update_turret(double x, double y, double heading){

        shooterWorldX = x + (shooterOffset * Math.cos(heading));
        shooterWorldY = y + (shooterOffset * Math.sin(heading));


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
        double hiveAngleOffset = TeleOp_Methods.xGoalOffset(goalX, x, goalY, y);


        target_angle = Math.atan2(goalY - shooterWorldY, goalX - shooterWorldX + hiveAngleOffset);
        target_angle = AngleUnit.normalizeRadians(target_angle);
        heading = AngleUnit.normalizeRadians(heading);

        relative_angle = Math.toDegrees(target_angle - heading);
        relative_angle = AngleUnit.normalizeDegrees(relative_angle) + offset;
        relative_angle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, relative_angle));
        target_position = Range.scale(relative_angle, MIN_ANGLE, MAX_ANGLE, MIN_POS, MAX_POS);

        if(!Globals.FAILSAFE_MODE){
            moveTo(target_position);
        }
        else{
            moveTo(failsafe_position);
        }
    }

    public void update_sotm(double y, double heading){

        double predX = Globals.predX;
        double predY = Globals.predY;

        shooterWorldX = predX + (shooterOffset * Math.cos(heading));
        shooterWorldY = predY + (shooterOffset * Math.sin(heading));


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


        double hiveAngleOffset = TeleOp_Methods.xGoalOffset(goalX, predX, goalY, predY);
        target_angle = Math.atan2(goalY - shooterWorldY, goalX - shooterWorldX + hiveAngleOffset);
        target_angle = AngleUnit.normalizeRadians(target_angle);
        heading = AngleUnit.normalizeRadians(heading);

        // Normalize and Scale
        relative_angle = Math.toDegrees(AngleUnit.normalizeRadians(target_angle - heading)) + offset;
//        relative_angle = AngleUnit.normalizeDegrees(relative_angle + 180);   //pt turela pe invers
        ///am comentat o ca nu mai avem turela la spate
        // Ensure relative_angle is within your physical hardware limits
        relative_angle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, relative_angle));

        target_position = Range.scale(relative_angle, MIN_ANGLE, MAX_ANGLE, MIN_POS, MAX_POS);


        if (!Globals.FAILSAFE_MODE)
            moveTo(target_position);
        else
            moveTo(failsafe_position);

    }

    public void moveTo(double x){
        servo_left.setPosition(x);
        servo_right.setPosition(x);
    }

}
