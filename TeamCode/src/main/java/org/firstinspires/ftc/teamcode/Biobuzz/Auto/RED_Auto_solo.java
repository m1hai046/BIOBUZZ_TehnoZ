package org.firstinspires.ftc.teamcode.Biobuzz.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Shooter;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Turret;
import org.firstinspires.ftc.teamcode.Biobuzz.Useful.Globals;
import org.firstinspires.ftc.teamcode.pedro.Constants;


@Configurable
@Autonomous(name="RED_Auto_solo", group="Auto")

public class RED_Auto_solo extends LinearOpMode {

    private final Pose startPose = new Pose(8.5, 24.5, Math.toRadians(270));
    private final Pose collectgarden = new Pose(8.5, 9.5, Math.toRadians(272));
    private final Pose shootgardenStart = new Pose(8, 8, Math.toRadians(0));
    private final Pose shootgarden = new Pose(48, 118, Math.toRadians(0));
    private final Pose shootgardenControl1 = new Pose(29, 90, Math.toRadians(0));
    private final Pose collectflowerStart = new Pose(48, 118, Math.toRadians(90));
    private final Pose collectflower = new Pose(48, 130, Math.toRadians(90));
    private final Pose shootflowerStart = new Pose(47, 130, Math.toRadians(90));
    private final Pose shootflower = new Pose(52, 121, Math.toRadians(90));
    private final Pose parkStart = new Pose(47, 130, Math.toRadians(90));
    private final Pose park = new Pose(12.6969, 108.8613, Math.toRadians(90));

    public Follower follower;
    public ElapsedTime pathTimer;
    public IntakeTransfer intakeTransfer;
    public Shooter shooter;
    public Turret turret;
    int pathState = 0;

    public void runOpMode() throws InterruptedException {

        Globals.alliance = Globals.Alliance.RED;
        Globals.faze = Globals.FAZE.AUTO;

        intakeTransfer = new IntakeTransfer(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);

        pathTimer = new ElapsedTime();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        waitForStart();
        pathTimer.reset();
        Shooter.rpmOffset -= 40;

        turret.update_turret(
                startPose.x(),
                startPose.y(),
                startPose.heading()
        );
        shooter.update_shooter(
                startPose.x(),
                startPose.y()
        );

        intakeTransfer.RampaSus();


        while (opModeIsActive() && !isStopRequested()) {

            follower.update();
            Pose currentPose = follower.pose();


            shooter.update_shooter(
                    currentPose.x(),
                    currentPose.y()
            );

            turret.update_turret(
                    currentPose.x(),
                    currentPose.y(),
                    currentPose.heading()
            );

            autonomousPathUpdate();
        }

    }

    public Path collectgarden() {
        return Paths.line(startPose, collectgarden).linear(startPose, collectgarden);
    }

    public Path shootgarden() {
        return Paths.curve(shootgardenStart, shootgardenControl1, shootgarden).reverseTangent();
    }
    public Path collectflower() {
        return Paths.line(collectflowerStart, collectflower).linear(collectflowerStart, collectflower);
    }
    public Path shootflower() {
        return Paths.line(shootflowerStart, shootflower).linear(shootflowerStart, shootflower);
    }
    public Path park() {
        return Paths.line(parkStart, park).reverseTangent();
    }


    public void setPathState(int n){
        pathTimer.reset();
        pathState = n;

    }
    public void autonomousPathUpdate(){
        switch(pathState){

            case 0:
                if (!follower.following()){
                    shooter.state = Shooter.State.RUNNING;
                    intakeTransfer.RampaSus();
                    intakeTransfer.Off();
                    setPathState(11);
                }
                break;

            case 11:///da shoot la preload
                if (!follower.following() && pathTimer.milliseconds() > 2000){
                    intakeTransfer.Shoot();
                    shooter.state = Shooter.State.SHOOTING;
                    setPathState(1);
                }
                break;


            case 1:
                if (shooter.state != Shooter.State.SHOOTING){
                    intakeTransfer.Collect();
                    follower.follow(collectgarden());
                    setPathState(2);
                }
                break;



            case 2:
                if (pathTimer.milliseconds() > 3000){
                    follower.follow(shootgarden());
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.following() && pathTimer.milliseconds() > 4000){
                    intakeTransfer.Shoot();
                    shooter.state = Shooter.State.SHOOTING;
                    setPathState(3);
                }
                break;

            case 3:
                if (shooter.state != Shooter.State.SHOOTING){
                    intakeTransfer.RampaJos();
                    intakeTransfer.Collect();
                    follower.follow(collectflower());
                    setPathState(4);
                }
                break;

            case 4:
                if ((!follower.following() && pathTimer.milliseconds() > 2800) ||  pathTimer.milliseconds() > 3000){
                    follower.follow(shootflower());
                    Shooter.rpmOffset -=30;
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.following() && pathTimer.milliseconds() > 3000){
                    intakeTransfer.RampaSus();
                    intakeTransfer.Shoot();
                    shooter.state = Shooter.State.SHOOTING;
                    setPathState(5);
                }
                break;

            case 5:
                if (shooter.state != Shooter.State.SHOOTING){
                    follower.follow(park());
                    intakeTransfer.RampaSus();
                    intakeTransfer.Off();
                    setPathState(-1);
                }
                break;


        }
    }
}