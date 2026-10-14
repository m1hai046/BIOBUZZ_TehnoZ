package org.firstinspires.ftc.teamcode.Biobuzz.TeleOp;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Shooter;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Turret;
import org.firstinspires.ftc.teamcode.Biobuzz.Useful.Globals;
import org.firstinspires.ftc.teamcode.Biobuzz.Useful.TeleOp_Methods;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@Configurable
@TeleOp (name = "RED_TeleOp_no_sotm_no_pedro", group = "TeleOp")
public class RED_TeleOp extends LinearOpMode {

    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public IntakeTransfer intakeTransfer;
    public Turret turret;
    public Shooter shooter;
    public ElapsedTime shootTimer;
    public ElapsedTime loops = new ElapsedTime();
    public Follower follower;
    public TeleOp_Methods teleOp_Methods;



    public static Pose startingPose = new Pose(9, 9, Math.toRadians(90));///de vazut


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
        teleOp_Methods = new TeleOp_Methods(intakeTransfer, shooter, follower, startingPose);

        shootTimer = new ElapsedTime();
        loops = new ElapsedTime();

        follower = Constants.create(hardwareMap);

        follower.setPose(startingPose);///de vazut
        follower.update();

        Gamepad currentG1 = new Gamepad();
        Gamepad previousG1 = new Gamepad();
        Gamepad currentG2 = new Gamepad();
        Gamepad previousG2 = new Gamepad();

        teleOp_Methods.robotState = TeleOp_Methods.RobotState.DEFAULT;

        frontLeft  = hardwareMap.get(DcMotorEx.class, "FLM");
        frontRight = hardwareMap.get(DcMotorEx.class, "FRM");
        backLeft   = hardwareMap.get(DcMotorEx.class, "BLM");
        backRight  = hardwareMap.get(DcMotorEx.class, "BRM");


        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotorEx m : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }


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


            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            Pose currentPose = follower.pose();
            Velocity velocity = follower.velocity(); // Field-centric velocity

            shooter.update_shooter(
                    currentPose.x(),
                    currentPose.y()
            );

            turret.update_turret(
                    currentPose.x(),
                    currentPose.y(),
                    currentPose.heading()
            );



            teleOp_Methods.handleControls(currentG1, previousG1, currentG2, previousG2);
            follower.update();
            handleMecanumDrive();
            teleOp_Methods.runStateMachine();
            teleOp_Methods.update_telemetry(telemetry, loops);
            loops.reset();

        }
    }

    public void handleMecanumDrive() {
        double forward = -gamepad1.left_stick_y;
        double strafe  = gamepad1.left_stick_x * 1.1;   // + = dreapta
        double turn    = Math.signum(gamepad1.right_stick_x)
                * Math.pow(Math.abs(gamepad1.right_stick_x), 2.0)
                * 0.5;                            // + = rotire în sensul acelor de ceasornic

        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // Normalizare, ca să nu depășim 1.0 și să păstrăm proporțiile
        double max = Math.max(1.0,
                Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                        Math.max(Math.abs(bl), Math.abs(br))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }



}
