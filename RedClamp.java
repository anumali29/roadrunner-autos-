package org.firstinspires.ftc.teamcode.monkeyballs;


// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class RedClamp extends LinearOpMode{

    public class Lift {
        public DcMotor leftSlide;
        public DcMotor rightSlide;

        public Lift() {
            leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
            rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");
            leftSlide.setTargetPosition(0);
            rightSlide.setTargetPosition(0);
            leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class MoveSlide implements Action {
            private int targetPosition;
            private boolean initialized = false;

            public MoveSlide(int targetPosition) {
                this.targetPosition = targetPosition;
                new SleepAction(1);
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    leftSlide.setTargetPosition(targetPosition);
                    rightSlide.setTargetPosition(targetPosition);
                    leftSlide.setPower(0.7);
                    rightSlide.setPower(0.7);
                    initialized = true;
                }
                packet.put("Slide Height", leftSlide.getCurrentPosition());
                return leftSlide.isBusy();
            }

        }

        public Action slideTo(int position){
            return new MoveSlide(position);
        }
    }

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(15, -64, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Mechanisms mech = new Mechanisms(this, hardwareMap);
        Lift lift = new Lift();

        //scores preload
        Action preload = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(10,-36))
                .waitSeconds(2)
                .build();

        //grabs from ground
        Action grab2 = drive.actionBuilder(new Pose2d(10, -36, Math.toRadians(90)))
                .lineToY(-45)
                .strafeToLinearHeading(new Vector2d(48,-38), Math.toRadians(270))
                .waitSeconds(1)
                .build();

        //give to human and pick up other from human
        Action pickup2 = drive.actionBuilder(new Pose2d(48, -38, Math.toRadians(270)))
                .strafeTo(new Vector2d(48,-48))
                .turn(Math.toRadians(180))
                .waitSeconds(3)
                .build();

        //score2
        Action score2 = drive.actionBuilder(new Pose2d(48, -48, Math.toRadians(180)))
                .strafeTo(new Vector2d(5,-36))
                .waitSeconds(2)
                .build();

        //picks from human
        Action pickup3 = drive.actionBuilder(new Pose2d(5, -36, Math.toRadians(180)))
                .strafeTo(new Vector2d(48,-48))
                .waitSeconds(1.5)
                .build();

        //score3
        Action score3 = drive.actionBuilder(new Pose2d(48, -48, Math.toRadians(180)))
                .strafeTo(new Vector2d(0,-36))
                .waitSeconds(2)
                .build();

        //park
        Action park = drive.actionBuilder(new Pose2d(0, -36, Math.toRadians(180)))
                .strafeTo(new Vector2d(42,-58))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        preload,
                        //score it
                        grab2,
                        //grab sample
                        pickup2,
                        //give to human player
                        //pickup specimen from human player
                        score2,
                        //score it
                        pickup3,
                        //pickup specimen from human player
                        score3,
                        //score it
                        park
                )
        );

    }
}
