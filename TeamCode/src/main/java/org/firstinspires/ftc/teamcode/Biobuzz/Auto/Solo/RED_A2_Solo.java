package org.firstinspires.ftc.teamcode.Biobuzz.Auto.Solo;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Turret.Turret;
import org.firstinspires.ftc.teamcode.Biobuzz.Useful.Globals;
import org.firstinspires.ftc.teamcode.Biobuzz.Useful.Pose_Storage;
import org.firstinspires.ftc.teamcode.pedro.Constants;


@Configurable
@Autonomous(name="RED_A2_Solo", group="Solo")

public class RED_A2_Solo extends LinearOpMode {

    public Follower follower;
    public ElapsedTime pathTimer;
    public IntakeTransfer intakeTransfer;
    public Shooter shooter;
    public Turret turret;

    public void runOpMode() throws InterruptedException {

        Globals.alliance = Globals.Alliance.RED;
        Globals.faze = Globals.FAZE.AUTO;
        Paths_Auto_Solo.buildPoses();

        intakeTransfer = new IntakeTransfer(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);

        pathTimer = new ElapsedTime();
        follower = Constants.create(hardwareMap);
        Paths_Auto_Solo.initPathsAutoSolo(follower, pathTimer, intakeTransfer, shooter, turret);
        follower.setPose(Paths_Auto_Solo.startPose);
        follower.update();

        turret.update_turret(Paths_Auto_Solo.startPose.x(), Paths_Auto_Solo.startPose.y(), Paths_Auto_Solo.startPose.heading());
        intakeTransfer.RampaSus();

        waitForStart();
        pathTimer.reset();


        while (opModeIsActive() && !isStopRequested()) {

            follower.update();
            Pose currentPose = follower.pose();

            turret.update_turret(
                    currentPose.x(),
                    currentPose.y(),
                    currentPose.heading()
            );

            shooter.update_shooter(
                    currentPose.x(),
                    currentPose.y()
            );

            Paths_Auto_Solo.autonomousPathUpdate();
            Pose_Storage.lastX = currentPose.x();
            Pose_Storage.lastY = currentPose.y();
            Pose_Storage.lastHeading = currentPose.heading();
        }
    }
}