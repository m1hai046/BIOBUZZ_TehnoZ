package org.firstinspires.ftc.teamcode.Biobuzz.TeleOp;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Shooter;
import org.firstinspires.ftc.teamcode.Biobuzz.Useful.Globals;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@Configurable
@TeleOp (name = "parnaie_teleop", group = "TeleOp")
public class parnaie_teleop extends LinearOpMode {

    public IntakeTransfer intakeTransfer;
    public Shooter shooter;
    public ElapsedTime shootTimer;
    public RobotState robotState;
    public ElapsedTime loops = new ElapsedTime();
    //public Follower follower;

    private static final double trigger_threshold = 0.3;
    public double SHOOT_DURATION_MS = 800.0;
    public static Pose startingPose = new Pose(55, 39, Math.toRadians(90));///de vazut


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
        shooter = new Shooter(hardwareMap);


        shootTimer = new ElapsedTime();
        loops = new ElapsedTime();

        //follower = Constants.create(hardwareMap);

        //follower.setPose(startingPose);///de vazut
        ///follower.update();



        Gamepad currentG1 = new Gamepad();
        Gamepad previousG1 = new Gamepad();
        Gamepad currentG2 = new Gamepad();
        Gamepad previousG2 = new Gamepad();

        robotState = RobotState.DEFAULT;

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("FLM");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("BLM");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("FRM");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("BRM");

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        while (opModeIsActive() && !isStopRequested()){




            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x * 0.5;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            previousG1.copy(currentG1);
            currentG1.copy(gamepad1);
            previousG2.copy(currentG2);
            currentG2.copy(gamepad2);



            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            //Pose currentPose = follower.pose();
            //Velocity velocity = follower.velocity(); // Field-centric velocity

            shooter.parnaie();



            handleControls(currentG1, previousG1, currentG2, previousG2);
            //follower.update();
            runStateMachine();
            update_telemetry();
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

//        if (current.dpad_down && !previous.dpad_down) {
//            follower.setPose(startingPose);
//        }



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
                intakeTransfer.Off();
                break;

            case INTAKE:
                intakeTransfer.Collect();
                break;

            case REVERSE:
                intakeTransfer.Reverse();
                break;

            case SHOOT:
                intakeTransfer.Shoot();
                break;
        }
    }
//    public void handleMecanumDrive(){
//        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
//        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//        double rx = gamepad1.right_stick_x * 0.5;
//
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double frontLeftPower = (y + x + rx) / denominator;
//        double backLeftPower = (y - x + rx) / denominator;
//        double frontRightPower = (y - x - rx) / denominator;
//        double backRightPower = (y + x - rx) / denominator;
//        motorFrontLeft.setPower(frontLeftPower);
//        motorBackLeft.setPower(backLeftPower);
//        motorFrontRight.setPower(frontRightPower);
//        motorBackRight.setPower(backRightPower);
//    }

    public void update_telemetry(){
        telemetry.addLine("-------- ROBOT STATE --------");
        telemetry.addData("  State", robotState.name());



        if (robotState == RobotState.SHOOT)
            telemetry.addData("  Shoot timer", "%.0f / %.0f ms",
                    shootTimer.milliseconds(), SHOOT_DURATION_MS);

        telemetry.addLine("-------- INTAKE/TRANSFER --------");
        telemetry.addData("  Intake power", "%.2f", intakeTransfer.motorIntake.getPower());
        telemetry.addData("  Transfer power", "%.2f", intakeTransfer.motorTransfer.getPower());

//        telemetry.addLine("-------- LOCALIZARE --------");
//        telemetry.addData("  Pose X", "%.1f", follower.pose().x());
//        telemetry.addData("  Pose Y", "%.1f", follower.pose().y());
//        telemetry.addData("  Heading", "%.1f°", Math.toDegrees(follower.pose().heading()));

        //telemetry.addData("distance from goal ", shooter.distance_from_goal);


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
