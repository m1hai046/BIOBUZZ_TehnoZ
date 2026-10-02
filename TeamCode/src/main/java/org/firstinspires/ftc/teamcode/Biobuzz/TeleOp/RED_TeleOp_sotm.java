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

@Configurable
@TeleOp (name = "RED_TeleOp_sotm", group = "TeleOp")
public class RED_TeleOp_sotm extends LinearOpMode {

    public IntakeTransfer intakeTransfer;
    public Turret turret;
    public Shooter shooter;
    public RobotState robotState;
    public ElapsedTime shootTimer;
    public ElapsedTime loops = new ElapsedTime();
    public Follower follower;

    private static final double trigger_threshold = 0.3;
    public double SHOOT_DURATION_MS = 800.0;
    public static Pose startingPose = new Pose(15.5, 144.0 - 80.0, Math.toRadians(180));///de vazut


    public List<LynxModule> allHubs;

    @Override
    public void runOpMode(){

        Globals.alliance = Globals.Alliance.RED;
        Globals.faze = Globals.FAZE.TELEOP;

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        telemetry.setMsTransmissionInterval(11);


        intakeTransfer = new IntakeTransfer(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);

        shootTimer = new ElapsedTime();
        loops = new ElapsedTime();

        follower = Constants.create(hardwareMap);

        follower.setPose(startingPose);
        follower.update();

        Gamepad currentG1 = new Gamepad();
        Gamepad previousG1 = new Gamepad();
        Gamepad currentG2 = new Gamepad();
        Gamepad previousG2 = new Gamepad();

        robotState = RobotState.DEFAULT;


        waitForStart();


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        while (opModeIsActive() && !isStopRequested()){

            previousG1.copy(currentG1);
            currentG1.copy(gamepad1);
            previousG2.copy(currentG2);
            currentG2.copy(gamepad2);
            handleMecanumDrive();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            Pose currentPose = follower.pose();
            Velocity velocity = follower.velocity(); // Field-centric velocity

            shooter.update_sotm(
                    currentPose.x(),
                    currentPose.y(),
                    velocity.toVector2D().x(),
                    velocity.toVector2D().y()
            );

            turret.update_sotm(
                    currentPose.x(),
                    currentPose.y(),
                    currentPose.heading(),
                    velocity.toVector2D().x(),
                    velocity.toVector2D().y()
            );



            handleControls(currentG1, previousG1, currentG2, previousG2);
            follower.update();
            runStateMachine();
            update_telemetry(shooter.distance_from_goal);
            loops.reset();

        }
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
                shootTimer.reset();
            }
        }

        if (current.dpad_down && !previous.dpad_down) {
            follower.setPose(startingPose);
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
                shooter.state = Shooter.State.RUNNING;
                intakeTransfer.Off();
                break;

            case INTAKE:
                intakeTransfer.Collect();
                break;

            case REVERSE:
                shooter.state = Shooter.State.RUNNING;
                intakeTransfer.Reverse();
                break;

            case SHOOT:
                shooter.state = Shooter.State.SHOOTING;
                intakeTransfer.Shoot();
                if (shooter.state == Shooter.State.RUNNING) {
                    robotState = RobotState.INTAKE;
                }
                break;
        }
    }
    public void handleMecanumDrive(){
        double turn = Math.signum(-gamepad1.right_stick_x)
                * Math.pow(Math.abs(-gamepad1.right_stick_x), 2.0)
                * 0.5;

        follower.drivetrain.drive(
                new DrivePowers(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x * 1.1,
                        turn
                ),
                true
        );

    }

    public void update_telemetry(double dist){
        telemetry.addLine("-------- ROBOT STATE --------");
        telemetry.addData("  State", robotState.name());

        telemetry.addLine("-------- SHOOTER --------");
        telemetry.addData(" vel shooter ", shooter.motor_shooter.getVelocity());
        telemetry.addData(" RPM target", "%.1f", Shooter.TARGET_VELOCITY);


        if (robotState == RobotState.SHOOT)
            telemetry.addData("  Shoot timer", "%.0f / %.0f ms",
                    shootTimer.milliseconds(), SHOOT_DURATION_MS);

        telemetry.addLine("-------- TURELA --------");
        telemetry.addData("  Unghi target", "%.1f°", Turret.target_position);
        telemetry.addData("  Rate", "%.1f°/s", Turret.target_angle);

        telemetry.addLine("-------- INTAKE/TRANSFER --------");
        telemetry.addData("  Intake power", "%.2f", intakeTransfer.motorIntake.getPower());
        telemetry.addData("  Transfer power", "%.2f", intakeTransfer.motorTransfer.getPower());

        telemetry.addLine("-------- LOCALIZARE --------");
        telemetry.addData("  Pose X", "%.1f", follower.pose().x());
        telemetry.addData("  Pose Y", "%.1f", follower.pose().y());
        telemetry.addData("  Heading", "%.1f°", Math.toDegrees(follower.pose().heading()));

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
