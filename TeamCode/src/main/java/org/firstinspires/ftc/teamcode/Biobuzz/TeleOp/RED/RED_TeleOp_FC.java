package org.firstinspires.ftc.teamcode.Biobuzz.TeleOp.RED;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Drive;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Turret.Turret;
import org.firstinspires.ftc.teamcode.Biobuzz.Useful.Globals;
import org.firstinspires.ftc.teamcode.Biobuzz.Useful.Pose_Storage;
import org.firstinspires.ftc.teamcode.Biobuzz.Useful.TeleOp_Methods;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@Configurable
@TeleOp (name = "RED_TeleOp_FC", group = "FC")
public class RED_TeleOp_FC extends LinearOpMode {
    public IntakeTransfer intakeTransfer;
    public Turret turret;
    public Shooter shooter;
    public Drive drive;
    public ElapsedTime shootTimer;
    public ElapsedTime loops = new ElapsedTime();
    public Follower follower;
    public TeleOp_Methods teleOp_Methods;
    public static Pose resetPose = new Pose(9, 9, Math.toRadians(90));

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
        drive = new Drive();

        shootTimer = new ElapsedTime();
        loops = new ElapsedTime();

        follower = Constants.create(hardwareMap);

        Pose startingPose = new Pose(
                Pose_Storage.lastX,
                Pose_Storage.lastY,
                Pose_Storage.lastHeading
        );

        follower.setPose(startingPose);///de vazut
        follower.update();
        teleOp_Methods = new TeleOp_Methods(intakeTransfer, shooter, follower, resetPose);
        teleOp_Methods.robotState = TeleOp_Methods.RobotState.DEFAULT;

        Gamepad currentG1 = new Gamepad();
        Gamepad previousG1 = new Gamepad();
        Gamepad currentG2 = new Gamepad();
        Gamepad previousG2 = new Gamepad();


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
            follower.update();

            Pose currentPose = follower.pose();
            Velocity velocity = follower.velocity(); // Field-centric velocity

            turret.update_turret(
                    currentPose.x(),
                    currentPose.y(),
                    currentPose.heading()
            );

            shooter.update_shooter(
                    currentPose.x(),
                    currentPose.y()
            );


            teleOp_Methods.handleControls(currentG1, previousG1, currentG2, previousG2);
            drive.pedro_FC(follower, gamepad1);
            teleOp_Methods.runStateMachine();
            teleOp_Methods.update_telemetry(telemetry, loops);
            loops.reset();

        }
    }
}
