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
    private DcMotor liftMotor;
    private Servo armServo;
    private Servo clawRotationServo;
    private Servo claw1;
    private Servo claw2;
    
    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(15, 64, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Mechanisms mech = new Mechanisms(this, hardwareMap);
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        armServo = hardwareMap.get(Servo.class, "armServo");
        clawRotationServo = hardwareMap.get(Servo.class "clawRotation");
         claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        if (isStopRequested()) return;
        // Actions

        Action moveToLiftPosition(int position){
            return new Action(){

            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if(!initialized){

                    liftMotor.setTargetPosition(position);
                    liftMotor.setPower(0.7);
                    initialized = true;
                }
                    packet.put("Lift Position", liftMotor.getCurrentPosition());
                    return liftMotor.isBusy();
                
            }
       };
  }
        Action moveArmAndClaw(double armPosition, double clawRotationPosition) {
            return new Action(){
                private boolean initialized = false;
                @Override
                public bollean run(@NonNull TelemetryPacket packet){
                    if(!initialized) { 
                        armServo.setPosition(armPosition);
                        clawRotationServo.setPosition(clawRotationPosition);
                        initialized = true;
                    }
                    packet.put("Arm Position", armServo.getPosition());
                    packet.put("Claw Rotation Position", clawRotationServo.getPosition());
                    return false;
                }
                
            };

        }
    Action setClawPosition(double position1, double position2){
        return new Action(){
            private boolean initialized = false;
            @Overrode
            public boolean run(@NonNull TelemetryPacket packet){
                if (!initialized){
                    claw1.setPosition(position1);
                    claw2.setPosition(position2);
                    initialized = true;
                }
                packet.put("Claw1 Position", claw1.getPosition());
                packet.put("Claw2 Position", claw2.getPosition());
                return false;
            }
            
        };
    
    }
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
                        moveToLiftPosition(-2000),
                        moveArmAndClaw(0.5, 0.3),
                        setClawPosition(0.65 ,0.35), //Open claw to release preloaded sample
                        grab2,
                        moveToLiftPosition(-350)
                        setClawPosition(0.4, 0.2
                        score2,
                        moveToLiftPosition(-2000),
                        setClawPosition(0.0, 0.8), // Open claw to release
                        pickup3,
                        setClawPosition(0.8, 0.1
                        score3,
                        moveToLiftPosition(-2000),
                        setClawPosition(0.0, 0.8),
                        pickup4,
                        moveToLiftPosition(-350),
                        setClawPosition(0.4, 0.2);
                        score4,
                        moveToLiftPosition(-2000),
                        setClawPosition(0.0, 0.8), // Open claw to release            
                        park,
                        () -> {
                            telemetry.addData("Status", "Parked");
                            telemetry.update();
                        }
                )
        );
    }
}
