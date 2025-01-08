package org.firstinspires.ftc.teamcode.monkeyballs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class MonkeyDrive extends LinearOpMode {

    ElapsedTime clawTimer = new ElapsedTime();

    public void runOpMode() {
        Drivetrain dt = new Drivetrain(this, hardwareMap);
        Mechanisms mech = new Mechanisms(this, hardwareMap);

        mech.armLeft.setPosition(0.6);
        mech.armRight.setPosition(0.6);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        clawTimer.startTime();

        while (opModeIsActive()) {
//            telemetry.addData("Status", "Running");
//            telemetry.update();

            double forward = (-1 * (gamepad1.left_stick_y));
            double strafe = (gamepad1.left_stick_x);
            double rotate = (gamepad1.right_stick_x);

            double FL = (0.8 * (forward + strafe + rotate));
            double FR = (0.8 * (forward - strafe - rotate));
            double BL = (0.8 * (forward - strafe + rotate));
            double BR = (0.8 * (forward + strafe - rotate));

            dt.leftFront.setPower(FL);
            dt.rightFront.setPower(FR);
            dt.leftRear.setPower(BL);
            dt.rightRear.setPower(BR);

            if(gamepad1.x){
                mech.heightToPickup();
            }

            if(gamepad1.y){
                mech.heightToFirstBasket();
            }

            if(gamepad1.b){
                mech.heightToSecondBasket();
            }

            //fine-tuning slide position go up
            if(gamepad2.dpad_up){
                mech.slideUp();
            }

            //fine-tuning slide position go down
            if(gamepad2.dpad_down){
                mech.slideDown();
            }

            //moving arm up gradually
            if(gamepad2.dpad_right){
                mech.armUp();
                telemetry.addData("leftPosition: ", mech.armLeft.getPosition());
                telemetry.update();
            }

            //moving arm down gradually
            if(gamepad2.dpad_left) {
                mech.armDown();
                telemetry.addData("leftPosition: ", mech.armLeft.getPosition());
                telemetry.update();
            }

            //moving claw up gradually (possibly re-zero to set a higher 1 value)
            if(gamepad2.right_bumper){
                mech.clawUp();
//                telemetry.addData("teehee", mech.claw.getPosition());
//                telemetry.update();
            }

            //moving claw down gradually
            if(gamepad2.left_bumper){
                mech.clawDown();
//                telemetry.addData("teehee", mech.claw.getPosition());
//                telemetry.update();
            }

            if(gamepad2.x){
                mech.clawOpen();
            }

            if(gamepad2.y){
                mech.clawClose();
            }

            if(gamepad2.right_trigger > 0.5){
                mech.spinToRight();
                telemetry.addData("rotate value", mech.clawRotation.getPosition());
                telemetry.update();
                //clawRotation is 0.25 at pickup and 0.95 to score
            }

            if(gamepad2.left_trigger > 0.5){
                mech.spinToLeft();
                telemetry.addData("rotate value", mech.clawRotation.getPosition());
                telemetry.update();
            }

        }

    }
}