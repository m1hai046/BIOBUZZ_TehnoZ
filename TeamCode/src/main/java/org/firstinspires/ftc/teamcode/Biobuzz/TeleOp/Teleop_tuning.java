package org.firstinspires.ftc.teamcode.Biobuzz.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Shooter;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Turret;
import org.firstinspires.ftc.teamcode.Biobuzz.Useful.Globals;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

import kotlin.internal.ContractsDsl;

@Configurable
@TeleOp (name = "Teleop_tuning", group = "TeleOp")
public class Teleop_tuning extends LinearOpMode {

    public static double left_pos = 0.5, right_pos = 0.7;
    public Turret turret;
    public Shooter shooter;
    public ElapsedTime shootTimer;
    public ElapsedTime loops = new ElapsedTime();

    private static final double trigger_threshold = 0.3;
    public double SHOOT_DURATION_MS = 800.0;
    public static Pose startingPose = new Pose(55, 39, Math.toRadians(90));
    /// de vazut


    public List<LynxModule> allHubs;

    @Override
    public void runOpMode() {

        Globals.alliance = Globals.Alliance.RED;
        Globals.faze = Globals.FAZE.TELEOP;

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry.setMsTransmissionInterval(11);


        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);

        shootTimer = new ElapsedTime();
        loops = new ElapsedTime();


        Gamepad currentG1 = new Gamepad();
        Gamepad previousG1 = new Gamepad();
        Gamepad currentG2 = new Gamepad();
        Gamepad previousG2 = new Gamepad();


        waitForStart();


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        while (opModeIsActive() && !isStopRequested()) {

            previousG1.copy(currentG1);
            currentG1.copy(gamepad1);
            previousG2.copy(currentG2);
            currentG2.copy(gamepad2);


            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }


            handleControls(currentG1, previousG1, currentG2, previousG2);
            update_telemetry();
            loops.reset();

        }
    }

    public void handleControls(Gamepad current, Gamepad previous, Gamepad current2, Gamepad previous2) {


        if (current.right_bumper && !previous.right_bumper){
            turret.servo_left.setPosition(right_pos);
            turret.servo_right.setPosition(right_pos);}
        if (current.left_bumper && !previous.left_bumper) {
            turret.servo_left.setPosition(left_pos);
            turret.servo_right.setPosition(left_pos);
        }
        if (current2.b && !previous2.b) Shooter.rpmOffset += 20;
        if (current2.a && !previous2.a) Shooter.rpmOffset -= 20;

        if (current2.right_bumper && !previous2.right_bumper)
            Turret.offset -= 1;
        if (current2.left_bumper && !previous2.left_bumper) {
            Turret.offset += 1;
        }

    }

    public enum RobotState {
        DEFAULT, // intake off, transfer off, gate closed, shooter off
        INTAKE,  // intake on, transfer on (low), gate closed, shooter on
        REVERSE, // intake on (reverse), transfer on (high→reverse), gate closed
        SHOOT,   // intake on, transfer on (high), gate open,  shooter on

    }




    public void update_telemetry(){
        telemetry.addLine("-------- ROBOT STATE --------");
        telemetry.addData("  failsafe", Globals.FAILSAFE_MODE);

        telemetry.addLine("-------- SHOOTER --------");
        telemetry.addData(" vel shooter ", shooter.motor_shooter.getVelocity());
        telemetry.addData(" RPM target", "%.1f", Shooter.TARGET_VELOCITY);




        telemetry.addLine("-------- TURELA --------");
        telemetry.addData("  Unghi target", "%.1f°", Turret.target_position);
        telemetry.addData("  Rate", "%.1f°/s", Turret.target_angle);


        telemetry.addData("distance from goal ", shooter.distance_from_goal);


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

