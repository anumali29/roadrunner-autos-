package org.firstinspires.ftc.teamcode.monkeyballs;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class BlueBucket extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(15, 64, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Mechanisms mech = new Mechanisms(this, hardwareMap);

        //scores preload
        Action preload = drive.actionBuilder(startPose)
                .splineToLinearHeading(new Pose2d(54,54, Math.toRadians(45)), Math.toRadians(0)) //score preload
                .build();

        //grabs first from ground
        Action grab2 = drive.actionBuilder(new Pose2d(54,54, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(48,48), Math.toRadians(90))
                .build();

        //score2
        Action score2 = drive.actionBuilder(new Pose2d(48, 48, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(54,54), Math.toRadians(45))
                .build();

        //grabs second from ground
        Action pickup3 = drive.actionBuilder(new Pose2d(54, 54, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(58,48), Math.toRadians(90))
                .build();

        //score3
        Action score3 = drive.actionBuilder(new Pose2d(58, 48, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(54,54), Math.toRadians(45))
                .build();

        //grabs third from ground
        Action pickup4 = drive.actionBuilder(new Pose2d(54, 54, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(48,24), Math.toRadians(180)) //pickup, gonna have to rotate claw
                .build();

        //score4
        Action score4 = drive.actionBuilder(new Pose2d(48, 24, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(54,54), Math.toRadians(45))
                .build();

        //park
        Action park = drive.actionBuilder(new Pose2d(54, 54, Math.toRadians(45)))
                .strafeTo(new Vector2d(54,13))
                .turn(Math.toRadians(135))
                .strafeTo(new Vector2d(24,12))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        preload,
                        //score it
                        grab2,
                        //grab sample closest to sub
                        score2,
                        //score it
                        pickup3,
                        //grab second sample from sub
                        score3,
                        //score it
                        pickup4,
                        //grab furthest sample
                        score4,
                        //score it
                        park
                )
        );
    }
}
