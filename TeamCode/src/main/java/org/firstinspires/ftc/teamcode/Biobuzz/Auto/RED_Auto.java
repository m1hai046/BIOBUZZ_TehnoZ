package org.firstinspires.ftc.teamcode.Biobuzz.Auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Shooter;
import org.firstinspires.ftc.teamcode.Biobuzz.Sisteme.Turret;
import org.firstinspires.ftc.teamcode.Biobuzz.Useful.Globals;
import org.firstinspires.ftc.teamcode.pedro.Constants;


@Configurable
@Autonomous(name="RED_Auto", group="Auto")

public class RED_Auto extends LinearOpMode {

    private final Pose startPose = new Pose(8.5, 24, Math.toRadians(270));
    private final Pose path1 = new Pose(8.5, 9, Math.toRadians(270));
    private final Pose point2Start = new Pose(8.5, 9, Math.toRadians(270));
    private final Pose point2 = new Pose(37, 114, Math.toRadians(250));
    private final Pose point3Start = new Pose(37, 114, Math.toRadians(62));
    private final Pose point3 = new Pose(45.6105, 126.063, Math.toRadians(82));
    private final Pose point4Start = new Pose(45.6105, 126.063, Math.toRadians(82));
    private final Pose point4 = new Pose(24.4119, 38.7704, Math.toRadians(72));
    private final Pose point5Start = new Pose(24.4119, 38.7704, Math.toRadians(72));
    private final Pose point5 = new Pose(16.8633, 42.4104, Math.toRadians(170));
    private final Pose point6Start = new Pose(16.8633, 42.4104, Math.toRadians(170));
    private final Pose point6 = new Pose(44.0294, 27.6226, Math.toRadians(150));
    private final Pose penStroke = new Pose(39.3699, 7.874, Math.toRadians(-103.2755));
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
            follower.update();

            autonomousPathUpdate();
        }

    }

    public Path path1() { return Paths.line(startPose, path1).linear(startPose, path1); }
    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return Paths.line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point5Start, point5).linear(point5Start, point5);
    }

    public Path path6() {
        return Paths.line(point6Start, point6).linear(point6Start, point6);
    }

    public Path penStroke() {
        return Paths.line(point6, penStroke).tangent();
    }


    public void setPathState(int n){
        pathTimer.reset();
        pathState = n;

    }
    public void autonomousPathUpdate(){
        switch(pathState){

                case 0:
                if (!follower.following()){
                    intakeTransfer.Off();
                    shooter.state = Shooter.State.STOPPED;z
                    setPathState(1);
                }
                break;

                case 1:
                if (!follower.following()){
                    follower.follow(path1());
                    setPathState(2);
                }
                break;

                case 2:
                if (!follower.following()){
                    follower.follow(path2());
                    setPathState(3);
                }
                break;

                case 3:
                if (!follower.following()){
                    follower.follow(path3());
                    setPathState(4);
                }
                break;

                case 4:
                if (!follower.following()){
                    follower.follow(path4());
                    setPathState(5);
                }
                break;

                case 5:
                if (!follower.following()){
                    follower.follow(path5());
                    setPathState(6);
                }
                break;

                case 6:
                if (!follower.following()){
                    follower.follow(path6());
                    setPathState(-1);
                }
                break;


        }
    }
}
