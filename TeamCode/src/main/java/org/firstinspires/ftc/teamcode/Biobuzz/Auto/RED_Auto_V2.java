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
@Autonomous(name="RED_Auto_V2", group="Auto")

public class RED_Auto_V2 extends LinearOpMode {

    private final Pose startPose = new Pose(8.5, 24, Math.toRadians(270));

    private final Pose shootPreload = new Pose(8.5, 24, Math.toRadians(90));
    private final Pose collectGarden = new Pose(8.5, 9, Math.toRadians(270));
    private final Pose shootGardenStart = new Pose(8.5, 9, Math.toRadians(270));
    private final Pose shootGarden = new Pose(40, 114, Math.toRadians(250));
    private final Pose collectshootFlower1Start = new Pose(40, 114, Math.toRadians(82));
    private final Pose collectshootFlower1 = new Pose(47, 128, Math.toRadians(90));
    private final Pose collectAlliance = new Pose(9, 106.6587, Math.toRadians(180));
    private final Pose collectAllianceSegment1Start = new Pose(9, 106.6587, Math.toRadians(90));
    private final Pose collectAllianceSegment1End = new Pose(9, 106.6587, Math.toRadians(180));
    private final Pose collectAllianceSegment2Heading = new Pose(9, 106.6587, Math.toRadians(180));
    private final Pose shootAlliance = new Pose(22, 34, Math.toRadians(100));
    private final Pose shootAlliance1Segment1Start = new Pose(22, 34, Math.toRadians(180));
    private final Pose shootAlliance1Segment1End = new Pose(22, 34, Math.toRadians(100));
    private final Pose shootAlliance1Segment2Start = new Pose(22, 34, Math.toRadians(100));
    private final Pose shootAlliance1Segment2End = new Pose(22, 34, Math.toRadians(100));
    private final Pose collectFlower2Start = new Pose(22, 34, Math.toRadians(130));
    private final Pose collectFlower2 = new Pose(12, 46.5, Math.toRadians(180));
    private final Pose shootFlower2Start = new Pose(12, 46.5, Math.toRadians(180));
    private final Pose shootFlower2 = new Pose(44, 27, Math.toRadians(150));
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

    public Path collectGarden() {
        return Paths.line(shootPreload, collectGarden).constant(collectGarden);
    }

    public Path shootGarden() {
        return Paths.line(shootGardenStart, shootGarden).linear(shootGardenStart, shootGarden);
    }

    public Path collectshootFlower1() {
        return Paths.line(collectshootFlower1Start, collectshootFlower1).linear(collectshootFlower1Start, collectshootFlower1);
    }

    public Path collectAlliance() {
        return Paths.line(collectshootFlower1, collectAlliance).heading
                (Interpolator.piecewise().until(0.8, Interpolator.linear(collectAllianceSegment1Start, collectAllianceSegment1End))
                        .until(1, Interpolator.constant(collectAllianceSegment2Heading)));
    }

    public Path shootAlliance() {
        return Paths.line(collectAlliance, shootAlliance).heading
                (Interpolator.piecewise().until(0.3, Interpolator.linear(shootAlliance1Segment1Start, shootAlliance1Segment1End))
                        .until(1, Interpolator.linear(shootAlliance1Segment2Start, shootAlliance1Segment2End)));
    }

    public Path collectFlower2() {
        return Paths.line(collectFlower2Start, collectFlower2).linear(collectFlower2Start, collectFlower2);
    }

    public Path shootFlower2() {
        return Paths.line(shootFlower2Start, shootFlower2).linear(shootFlower2Start, shootFlower2);
    }


    public void setPathState(int n){
        pathTimer.reset();
        pathState = n;

    }
    public void autonomousPathUpdate(){
        switch(pathState){

            case 0:
                if (!follower.following()){//da shoot la preload
                    shooter.state = Shooter.State.RUNNING;
                    intakeTransfer.RampaSus();
                    intakeTransfer.Off();
                    setPathState(111);
                }
                break;

            case 111:
                if (!follower.following() && pathTimer.milliseconds() > 1000){
                    shooter.state = Shooter.State.SHOOTING;
                    setPathState(1);
                }
                break;


            case 1:
                if (shooter.state != Shooter.State.SHOOTING){
                    intakeTransfer.Collect();
                    follower.follow(collectGarden());
                    setPathState(2);
                }
                break;


            case 2:
                if (!follower.following()){
                    follower.follow(shootGarden());
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.following()){
                    shooter.state = Shooter.State.SHOOTING;
                    setPathState(3);
                }
                break;

            case 3:
                if (shooter.state != Shooter.State.SHOOTING){
                    intakeTransfer.RampaJos();
                    follower.follow(collectshootFlower1());
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.following()){
                    shooter.state = Shooter.State.SHOOTING;
                    setPathState(4);
                }
                break;

            case 4:
                if (shooter.state != Shooter.State.SHOOTING){
                    intakeTransfer.RampaSus();
                    intakeTransfer.Collect();
                    follower.follow(collectAlliance());
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.following()){
                    follower.follow(shootAlliance());
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.following()){
                    shooter.state = Shooter.State.SHOOTING;
                    setPathState(6);
                }
                break;

            case 6:
                if (shooter.state != Shooter.State.SHOOTING){
                    intakeTransfer.RampaJos();
                    intakeTransfer.Collect();
                    follower.follow(collectFlower2());
                    setPathState(7);
                }
                break;


            case 7:
                if (!follower.following()){
                    follower.follow(shootFlower2());
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.following()){
                    shooter.state = Shooter.State.SHOOTING;
                    setPathState(-1);
                }
                break;
        }
    }
}