package org.firstinspires.ftc.teamcode.Biobuzz.Useful;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Utils;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Shooter;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Turret;

@Configurable
public class TeleOp_Methods {
    public RobotState robotState;
    private static final double trigger_threshold = 0.3;
    public double SHOOT_DURATION_MS = 800.0;
    public static double X_OFFSET_GAIN = 0.15, MAX_X_OFFSET = 0.0;

    public IntakeTransfer M_intakeTransfer;
    public Shooter M_shooter;
    public Follower M_follower;

    public Pose M_startingPose;

    public TeleOp_Methods(IntakeTransfer intakeTransfer, Shooter shooter,
                          Follower follower, Pose startingPose) {
        M_intakeTransfer = intakeTransfer;
        M_shooter = shooter;
        M_follower = follower;
        M_startingPose = startingPose;
    }

    public void handleControls(Gamepad current, Gamepad previous, Gamepad current2, Gamepad previous2){
        if (current.right_bumper && !previous.right_bumper) {
            robotState = (robotState == RobotState.INTAKE)
                    ? RobotState.DEFAULT : RobotState.INTAKE;
        }

        if (current.left_bumper && !previous.left_bumper) {
            robotState = (robotState == RobotState.REVERSE)
                    ? RobotState.DEFAULT : RobotState.REVERSE;
        }

        if (current.touchpad && !previous.touchpad) {
            Globals.FAILSAFE_MODE = !Globals.FAILSAFE_MODE;
        }

        if (current.right_trigger > trigger_threshold
                && previous.right_trigger <= trigger_threshold) {
            if (robotState != RobotState.SHOOT) {
                robotState = RobotState.SHOOT;
                M_shooter.state = Shooter.State.SHOOTING;
            }
        }

        if (current.dpad_down && !previous.dpad_down) {
            M_follower.setPose(M_startingPose);
            Turret.offset = 0;
            Shooter.rpmOffset = 0;
        }

        if (current2.b && !previous2.b) Shooter.rpmOffset += 20;
        if (current2.a && !previous2.a) Shooter.rpmOffset -= 20;

        if (current2.right_bumper && !previous2.right_bumper)
            Turret.offset -= 1;
        if (current2.left_bumper && !previous2.left_bumper) {
            Turret.offset += 1;
        }

    }
    public enum RobotState{
        DEFAULT, // intake off, transfer off, gate closed, shooter off
        INTAKE,  // intake on, transfer on (low), gate closed, shooter on
        REVERSE, // intake on (reverse), transfer on (high→reverse), gate closed
        SHOOT,   // intake on, transfer on (high), gate open,  shooter on

    }

    public void runStateMachine(){
        switch (robotState){

            case DEFAULT:
                M_shooter.state = Shooter.State.RUNNING;
                M_intakeTransfer.Off();
                break;

            case INTAKE:
                M_intakeTransfer.Collect();
                if(IntakeTransfer.hasArtifact() == true){
                    robotState = RobotState.DEFAULT;
                }

                break;

            case REVERSE:
                M_shooter.state = Shooter.State.RUNNING;
                M_intakeTransfer.Reverse();
                break;

            case SHOOT:
                M_intakeTransfer.Shoot();
                if (M_shooter.state == Shooter.State.RUNNING) {
                    robotState = RobotState.INTAKE;
                }
                IntakeTransfer.confirmare = 0;
                break;
        }
    }


    ///folosita la shooter si turela
    public static double xGoalOffset(double goalX, double robotX, double goalY, double robotY){
        double lateral = goalX - robotX;
        double forward = goalY - robotY;
        double hypot = Math.hypot(lateral, forward);

        if (hypot < 0.01) return 0;

        double sinVal = lateral / hypot;
        if (robotY >70.7)
        {
            return Utils.clamp(sinVal * X_OFFSET_GAIN, MAX_X_OFFSET, -MAX_X_OFFSET);

        }
        else {
            return Utils.clamp(sinVal * X_OFFSET_GAIN, -MAX_X_OFFSET, MAX_X_OFFSET);
        }
    }



    public void update_telemetry(Telemetry telemetry, ElapsedTime loops){
        telemetry.addLine("-------- ROBOT STATE --------");
        telemetry.addData(" State", robotState.name());
        telemetry.addData(" failsafe", Globals.FAILSAFE_MODE);

        telemetry.addLine("-------- SHOOTER --------");
        telemetry.addData(" vel shooter ", M_shooter.motor_shooter.getVelocity());
        telemetry.addData(" RPM target", "%.1f", Shooter.TARGET_VELOCITY);
        telemetry.addData(" hood angle", Shooter.HOOD_ANGLE);
        telemetry.addData(" goalX", Shooter.goalX);
        telemetry.addData(" goalY", Shooter.goalY);
        telemetry.addData(" voltaj_shooter", M_shooter.shooter_voltage.getVoltage());

//        telemetry.addData(" gate pos", Shooter.);


        if (robotState == TeleOp_Methods.RobotState.SHOOT)
            telemetry.addData("  Shoot timer", "%.0f / %.0f ms",
                    M_shooter.shootTimer.milliseconds(), SHOOT_DURATION_MS);

        telemetry.addLine("-------- TURELA --------");
        telemetry.addData(" Unghi target", "%.1f°", Turret.target_position);
        telemetry.addData(" Rate", "%.1f°/s", Turret.target_angle);
        telemetry.addData(" goalX", Turret.goalX);
        telemetry.addData(" goalY", Turret.goalY);



        telemetry.addLine("-------- INTAKE/TRANSFER --------");
        telemetry.addData("  Intake power", "%.2f", IntakeTransfer.motorIntake.getPower());
        telemetry.addData("  Transfer power", "%.2f", IntakeTransfer.motorTransfer.getPower());
        telemetry.addData("  has artifact", IntakeTransfer.hasArtifact());
        telemetry.addData("  amps", IntakeTransfer.getAmps());

        telemetry.addData("  powIntake", IntakeTransfer.powIntake);
        telemetry.addData("  powTransferCollect", IntakeTransfer.powTransferCollect);
        telemetry.addData("  powTransferShoot", IntakeTransfer.powTransferShoot);

        telemetry.addLine("-------- LOCALIZARE --------");
        telemetry.addData("  Pose X", "%.1f", M_follower.pose().x());
        telemetry.addData("  Pose Y", "%.1f", M_follower.pose().y());
        telemetry.addData("  Heading", "%.1f°", Math.toDegrees(M_follower.pose().heading()));

        telemetry.addData("distance from goal ", M_shooter.distance_from_goal);


        telemetry.addLine("-------- PROFILING --------");
        telemetry.addData("loop time ", loops.milliseconds());


        //        telemetryM.debug("--- Shooter ---");
//        telemetryM.addData("motor1RPM",      shooter.motor_shooter.getVelocity());
//        telemetryM.addData("motor2RPM",      shooter.motor_shooter_2.getVelocity());
//        telemetryM.addData("RPM",            shooter.getRpm(shooter.motor_shooter.getVelocity()));
//        telemetryM.addData("targetVelocity", shooter.TARGET_VELOCITY);
//        telemetryM.addData("Velocity",       shooter.motor_shooter.getVelocity());

//        telemetryM.addData("graph.shooter.target", shooter.TARGET_VELOCITY);
//        telemetryM.addData("graph.shooter.actual", shooter.motor_shooter.getVelocity());
//        telemetryM.addData("graph.shooter.pwr1",   shooter.motor_shooter.getPower());
//        telemetryM.addData("graph.shooter.pwr2",   shooter.motor_shooter_2.getPower());

//        telemetryM.update(telemetry);
        telemetry.update();
    }




}
