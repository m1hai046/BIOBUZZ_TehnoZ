package org.firstinspires.ftc.teamcode.Biobuzz.Sisteme;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Biobuzz.Useful.Globals;
import org.firstinspires.ftc.teamcode.Biobuzz.Useful.TeleOp_Methods;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

/// notes
///
/// trb testat offset l ala de unghi, ca eu sunt desyul de sigur ca nu trb doar pe axa X dar si pe Y
/// trb facuta poz reglaj sa fie in range si la gate lfl
/// trb tunat: pvs ul, pozitii gate, interpolari, deadzone si lookheadtime care e in globals
/// trb pus in teleop timer de reset pt cand dai shoot
///
///

@Configurable
public class Shooter {

    public VoltageSensor shooter_voltage;
    public static double TARGET_VELOCITY, HOOD_ANGLE, gateOpen = 0.8, gateClosed = 0.54;
    public static double kP = 0.005 , kV = 0.00036, kS = 0.12, nominalVoltage = 12.0 , error, SHOOT_TIME = 800;

    public static double FAILSAFE_VELOCITY = 1420.0, FAILSAFE_ANGLE = 0.8  , TARGET_STOPPED = 0.0;
    public boolean running = false, shooting = false;
    public static double goalX, goalY;

    public static double minHoodAngle = 0.8,  maxHoodAngle = 0.8, minShooterRpm = 1380.0, maxShooterRpm = 1530.0;
    public static double rpmOffset = 0.0;

    public State state;
    public State previousState = State.RUNNING;

    public ElapsedTime shootTimer;
    public double distance_from_goal;

    public CachingDcMotorEx motor_shooter, motor_shooter_2;
    public CachingServo gate, hood;
    public static double X_OFFSET_GAIN = 0.15, MAX_X_OFFSET = 0.0;

    public Shooter(HardwareMap hardwareMap){
        motor_shooter = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "MSHT"));
        motor_shooter_2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "MSHT2"));
        gate = new CachingServo(hardwareMap.get(Servo.class, "Gate"));
        hood = new CachingServo(hardwareMap.get(Servo.class, "Reglaj"));

        motor_shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_shooter_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor_shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_shooter_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor_shooter.setDirection(DcMotor.Direction.FORWARD);
        motor_shooter_2.setDirection(DcMotorSimple.Direction.REVERSE);

        motor_shooter.setCachingTolerance(0.001);
        motor_shooter_2.setCachingTolerance(0.001);

        state = State.RUNNING;
        TARGET_VELOCITY = TARGET_STOPPED;
        gate.setPosition(gateClosed);
        shooter_voltage = hardwareMap.voltageSensor.iterator().next();

        shootTimer = new ElapsedTime();

    }

    public void update_shooter(double x, double y){     /// poz x is y de la follower luate in teleop

        if (state == State.SHOOTING && previousState != State.SHOOTING) {
            shootTimer.reset();
        }
        previousState = state;

        double vel = motor_shooter.getVelocity();


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

        double hiveAngleOffset = xGoalOffset(goalX, x, goalY, y);
        distance_from_goal = Math.hypot(goalX - x + hiveAngleOffset, goalY - y);

        if (!Globals.FAILSAFE_MODE) {

            if (distance_from_goal <= 200 && distance_from_goal >= 10){
                TARGET_VELOCITY = optimalRpm(distance_from_goal) + rpmOffset;
                HOOD_ANGLE = optimalAngle(distance_from_goal);
            }
        }
        else {
            TARGET_VELOCITY = FAILSAFE_VELOCITY;
            HOOD_ANGLE = FAILSAFE_ANGLE;
        }

        error = TARGET_VELOCITY - vel;



        if (running) {

            double voltageScaling = nominalVoltage / shooter_voltage.getVoltage();

            double proportional = (kP * error);
            double feedforward = (kV * TARGET_VELOCITY) + kS;

            double pow = (proportional + feedforward) * voltageScaling;

            pow = Math.max(-1.0, Math.min(1.0, pow));

            motor_shooter.setPower(pow);
            motor_shooter_2.setPower(pow);
            hood.setPosition(HOOD_ANGLE);
        }


        switch (state){
            case RUNNING:
                shooting = false;
                running = true;
                gateClosed();
                break;

            case SHOOTING:
                shooting = true;
                running = true;
                if (Math.abs(error) <= 100) {
                    gateOpen();
                    if (shootTimer.milliseconds() > 800) {
                        state = State.RUNNING;
                    }
                }
                break;

            case STOPPED:
                shooting = false;
                running = false;
                motor_shooter.setPower(0.0);
                motor_shooter_2.setPower(0.0);
                gateClosed();
                break;

        }
    }

    public void update_sotm(double x, double y, double vx, double vy){
        ///partea de sotm aici

        if (state == State.SHOOTING && previousState != State.SHOOTING) {
            shootTimer.reset();
        }
        previousState = state;

        double vel = motor_shooter.getVelocity();


        double deadzone = Globals.deadZone;
        double cleanVx = (Math.abs(vx) < deadzone) ? 0 : vx;
        double cleanVy = (Math.abs(vy) < deadzone) ? 0 : vy;

        // 1. Prediction
        double lookAheadTime = Globals.look_ahead_time;
        double predX = x + (cleanVx * lookAheadTime);
        double predY = y + (cleanVy * lookAheadTime);

        Globals.predX = predX;
        Globals.predY = predY;


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


        double hiveAngleOffset = xGoalOffset(goalX, predX, goalY, predY);
        distance_from_goal = Math.hypot(goalX - predX + hiveAngleOffset, goalY - predY);

        if (!Globals.FAILSAFE_MODE) {

            if (distance_from_goal <= 200 && distance_from_goal >= 10){
                TARGET_VELOCITY = optimalRpm(distance_from_goal) + rpmOffset;
                HOOD_ANGLE = optimalAngle(distance_from_goal);
            }
        }
        else {
            TARGET_VELOCITY = FAILSAFE_VELOCITY;
            HOOD_ANGLE = FAILSAFE_ANGLE;
        }

        error = TARGET_VELOCITY - vel;

        if (running) {

            double voltageScaling = nominalVoltage / shooter_voltage.getVoltage();


            double proportional = (kP * error);
            double feedforward = (kV * TARGET_VELOCITY) + kS;

            double pow = (proportional + feedforward) * voltageScaling;

            pow = Math.max(-1.0, Math.min(1.0, pow));

            motor_shooter.setPower(pow);
            motor_shooter_2.setPower(pow);
            hood.setPosition(HOOD_ANGLE);
        }


        switch (state) {

            case RUNNING:
                shooting = false;
                running = true;
                gateClosed();
                break;

            case SHOOTING:
                shooting = true;
                running = true;
                if (Math.abs(error) <= 100) {
                    gateOpen();
                    if (shootTimer.milliseconds() > SHOOT_TIME) {///trb pus timer in teleop cand apesi pe buton sa isi dea reset
                        state = State.RUNNING;
                    }
                }
                break;

            case STOPPED:
                shooting = false;
                running = false;
                motor_shooter.setPower(0.0);
                motor_shooter_2.setPower(0.0);
                gateClosed();
                break;
        }
    }

    public enum State{
        RUNNING,
        SHOOTING,
        STOPPED
    }


    public void gateClosed(){
        gate.setPosition(gateClosed);
    }

    public void gateOpen(){
        gate.setPosition(gateOpen);
    }

    public double MSHT1getVelocity(){
        return motor_shooter.getVelocity();
    }

    public double MSHT2getVelocity(){
        return motor_shooter_2.getVelocity();
    }

//    public double optimalAngle(double dist) {
//        double angle = -0.0000183333 * Math.pow(dist, 3)
//                + 0.0035 * Math.pow(dist, 2)
//                - 0.215167 * dist
//                + 4.4;
//        return Utils.clamp(angle, minHoodAngle, maxHoodAngle);
//    }



//    public double optimalRpm(double dist) {
//        double rpm = 0.00333333 * Math.pow(dist, 3)
//                - 0.5 * Math.pow(dist, 2)
//                + 26.66667 * dist
//                + 940;
//        return Utils.clamp(rpm, minShooterRpm, maxShooterRpm);
//    }

    public double optimalAngle(double dist) {
        double angle = 0 * dist + 0.8;
        return Utils.clamp(angle, minHoodAngle, maxHoodAngle);
    }

    public double optimalRpm(double dist) {
        double rpm = -0.02 * Math.pow(dist, 3)
                + 2.7 * Math.pow(dist, 2)
                - 112 * dist
                + 2850;
        return Utils.clamp(rpm, minShooterRpm, maxShooterRpm);
    }

    public static double xGoalOffset(double goalX, double robotX, double goalY, double robotY){
        double lateral = robotX - goalX;
        double forward = robotY - goalY;
        double hypot = Math.hypot(lateral, forward);

        if (hypot < 0.01) return 0;

        double sinVal = lateral / hypot;

        return Utils.clamp(sinVal * X_OFFSET_GAIN, -MAX_X_OFFSET, MAX_X_OFFSET);
    }


}


