package org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Shooter;

import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Turret.Turret;
import org.firstinspires.ftc.teamcode.Biobuzz.Useful.Globals;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class Shooter {

    public VoltageSensor shooter_voltage;
    public boolean running = false, shooting = false;
    public static double goalX, goalY;
    public static double error, nominalVoltage = 12.0;
    public static double TARGET_VELOCITY, HOOD_ANGLE;

    public static double rpmOffset = 0.0;

    public State state;
    public State previousState = State.RUNNING;

    public ElapsedTime shootTimer;
    public double distance_from_goal;

    public CachingDcMotorEx motor_shooter;
    public CachingServo gate, hood;

    public Shooter(HardwareMap hardwareMap){
        motor_shooter = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "MSHT"));
        gate = new CachingServo(hardwareMap.get(Servo.class, "Gate"));
        hood = new CachingServo(hardwareMap.get(Servo.class, "Reglaj"));

        motor_shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor_shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor_shooter.setDirection(DcMotor.Direction.FORWARD);

        motor_shooter.setCachingTolerance(0.001);

        state = State.RUNNING;
        TARGET_VELOCITY = Shooter_Constants.TARGET_STOPPED;
        rpmOffset = 0;
        gate.setPosition(Shooter_Constants.gateClosed);
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

        distance_from_goal = Math.hypot(goalX - x + Turret.aimShift, goalY - y);

        if (!Globals.FAILSAFE_MODE) {

            if (distance_from_goal <= 200 && distance_from_goal >= 10){
                TARGET_VELOCITY = optimalRpm(distance_from_goal) + rpmOffset;
                HOOD_ANGLE = optimalAngle(distance_from_goal);
            }
        }
        else {
            TARGET_VELOCITY = Shooter_Constants.FAILSAFE_VELOCITY;
            HOOD_ANGLE = Shooter_Constants.FAILSAFE_ANGLE;
        }

        error = TARGET_VELOCITY - vel;



        if (running) {

            double voltageScaling = nominalVoltage / shooter_voltage.getVoltage();

            double proportional = (Shooter_Constants.kP * error);
            double feedforward = (Shooter_Constants.kV * TARGET_VELOCITY) + Shooter_Constants.kS;

            double pow = (proportional + feedforward) * voltageScaling;

            pow = Math.max(-1.0, Math.min(1.0, pow));

            motor_shooter.setPower(pow);
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
                if (Math.abs(error) <= Shooter_Constants.SHOOT_TOLERANCE) {
                    gateOpen();
                    if (shootTimer.milliseconds() > Shooter_Constants.SHOOT_TIME) {
                        state = State.RUNNING;
                    }
                }
                break;

            case STOPPED:
                shooting = false;
                running = false;
                motor_shooter.setPower(0.0);
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


        distance_from_goal = Math.hypot(goalX - predX + Turret.aimShift, goalY - predY);

        if (!Globals.FAILSAFE_MODE) {

            if (distance_from_goal <= 200 && distance_from_goal >= 10){
                TARGET_VELOCITY = optimalRpm(distance_from_goal) + rpmOffset;
                HOOD_ANGLE = optimalAngle(distance_from_goal);
            }
        }
        else {
            TARGET_VELOCITY = Shooter_Constants.FAILSAFE_VELOCITY;
            HOOD_ANGLE = Shooter_Constants.FAILSAFE_ANGLE;
        }

        error = TARGET_VELOCITY - vel;

        if (running) {

            double voltageScaling = nominalVoltage / shooter_voltage.getVoltage();


            double proportional = (Shooter_Constants.kP * error);
            double feedforward = (Shooter_Constants.kV * TARGET_VELOCITY) + Shooter_Constants.kS;

            double pow = (proportional + feedforward) * voltageScaling;

            pow = Math.max(-1.0, Math.min(1.0, pow));

            motor_shooter.setPower(pow);
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
                if (Math.abs(error) <= Shooter_Constants.SHOOT_TOLERANCE) {
                    gateOpen();
                    if (shootTimer.milliseconds() > Shooter_Constants.SHOOT_TIME) {///trb pus timer in teleop cand apesi pe buton sa isi dea reset
                        state = State.RUNNING;
                    }
                }
                break;

            case STOPPED:
                shooting = false;
                running = false;
                motor_shooter.setPower(0.0);
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
        gate.setPosition(Shooter_Constants.gateClosed);
    }

    public void gateOpen(){
        gate.setPosition(Shooter_Constants.gateOpen);
    }

    public double MSHT1getVelocity(){
        return motor_shooter.getVelocity();
    }


    public double optimalAngle(double dist) {
        double angle = 0 * dist + 0.8;
        return Utils.clamp(angle, Shooter_Constants.minHoodAngle, Shooter_Constants.maxHoodAngle);
    }

    public double optimalRpm(double dist) {
        double rpm = -0.000333333 * Math.pow(dist, 4)
                + 0.05 * Math.pow(dist, 3)
                - 2.56667 * Math.pow(dist, 2)
                + 56 * dist
                + 930;
        return Utils.clamp(rpm, Shooter_Constants.minShooterRpm, Shooter_Constants.maxShooterRpm);
    }

}