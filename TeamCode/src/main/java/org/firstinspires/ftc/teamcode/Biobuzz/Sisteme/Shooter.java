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
    public static double TARGET_VELOCITY, HOOD_ANGLE, gateOpen = 0.15, gateClosed = 0.5;
    public static double kP = 0.000375 , kV = 0.0004, kS = 0.05, nominalVoltage = 12.0 , error;
    public static double FAILSAFE_VELOCITY = 1000.0, FAILSAFE_ANGLE = 0.2, TARGET_STOPPED = 0.0;
    public boolean running = false, shooting = false;

    ///poz goal aprox, de tunat
    public static double X_GOAL_RED_SUP = 58, Y_GOAL_RED_SUP = 82, X_GOAL_RED_INF = 58, Y_GOAL_RED_INF = 58;
    public static double X_GOAL_BLUE_SUP = 84, Y_GOAL_BLUE_SUP = 82, X_GOAL_BLUE_INF = 84, Y_GOAL_BLUE_INF = 58;
    public static double goalX, goalY;

    public static double X_OFFSET_GAIN = 0.15, MAX_X_OFFSET = 4.0;
    public static double minHoodAngle = 0.3, maxHoodAngle = 0.8, minShooterRpm = 1650.0, maxShooterRpm = 1800.0;
    public static double rpmOffset = 0.0;

    public State state;
    public State previousState = State.RUNNING;

    public ElapsedTime shootTimer;
    public double distance_from_goal;

    public CachingDcMotorEx motor_shooter, motor_shooter_2;
    public CachingServo gate, hood;

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
                goalX = X_GOAL_RED_SUP;
                goalY = Y_GOAL_RED_SUP;
            }
            else{///inf
                goalX = X_GOAL_RED_INF;
                goalY = Y_GOAL_RED_INF;
            }
        }

        else {
            if (y >70.7) {///sup
                goalX = X_GOAL_BLUE_SUP;
                goalY = Y_GOAL_BLUE_SUP;
            }
            else{///inf
                goalX = X_GOAL_BLUE_INF;
                goalY = Y_GOAL_BLUE_INF;
            }
        }

        double hiveAngleOffset = xGoalOffset(goalX, x, goalY, y);
        distance_from_goal = Math.hypot(goalX - x + hiveAngleOffset, goalY - y);

        if (!Globals.FAILSAFE_MODE) {

            if (distance_from_goal <= 180 && distance_from_goal >= 40){
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

    public void parnaie(){
        running = true;
        double vel = motor_shooter.getVelocity();

        TARGET_VELOCITY = FAILSAFE_VELOCITY;
        HOOD_ANGLE = FAILSAFE_ANGLE;

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
                goalX = X_GOAL_RED_SUP;
                goalY = Y_GOAL_RED_SUP;
            }
            else{///inf
                goalX = X_GOAL_RED_INF;
                goalY = Y_GOAL_RED_INF;
            }
        }

        else {
            if (y >70.7) {///sup
                goalX = X_GOAL_BLUE_SUP;
                goalY = Y_GOAL_BLUE_SUP;
            }
            else{///inf
                goalX = X_GOAL_BLUE_INF;
                goalY = Y_GOAL_BLUE_INF;
            }
        }


        double hiveAngleOffset = xGoalOffset(goalX, predX, goalY, predY);
        distance_from_goal = Math.hypot(goalX - predX + hiveAngleOffset, goalY - predY);

        if (!Globals.FAILSAFE_MODE) {

            if (distance_from_goal <= 180 && distance_from_goal >= 40){
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
                    if (shootTimer.milliseconds() > 800) {///trb pus timer in teleop cand apesi pe buton sa isi dea reset
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

    public double optimalAngle(double dist){
        return Utils.clamp((0.02 * dist) - 0.3, minHoodAngle, maxHoodAngle);
    }
    public double optimalRpm(double dist){
        return Utils.clamp((0.0175 * Math.pow(dist, 3) - 2.35 * Math.pow(dist, 2) + 109.75 * dist + 0.0), minShooterRpm, maxShooterRpm);
    }

    public double xGoalOffset(double goalX, double robotX, double goalY, double robotY){
        double lateral = goalX - robotX;
        double forward = goalY - robotY;
        double hypot = Math.hypot(lateral, forward);

        if (hypot < 0.01) return 0;

        double sinVal = lateral / hypot;
        return Utils.clamp(sinVal * X_OFFSET_GAIN, -MAX_X_OFFSET, MAX_X_OFFSET);
    }



}


