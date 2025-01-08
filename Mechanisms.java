package org.firstinspires.ftc.teamcode.monkeyballs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
public class Mechanisms {
    public Servo armLeft;
    public Servo armRight;

    public DcMotor leftSlide;
    public DcMotor rightSlide;

    public Servo clawControl;
    public Servo clawRotation;

    public Servo claw;

    LinearOpMode opMode;

    public Mechanisms(LinearOpMode opMode, HardwareMap hardwareMap){
        armLeft = hardwareMap.get(Servo.class, "armLeft");
        armRight = hardwareMap.get(Servo.class, "armRight");

        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");

        clawControl = hardwareMap.get(Servo.class, "clawControl");
        clawRotation = hardwareMap.get(Servo.class, "clawRotation");

        claw = hardwareMap.get(Servo.class, "claw");

        this.opMode = opMode;

        claw.setDirection(Servo.Direction.REVERSE);

        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void heightToPickup(){
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        leftSlide.setTargetPosition(-350);
        rightSlide.setTargetPosition(-350);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(0.7);
        rightSlide.setPower(0.7);

        while (leftSlide.isBusy() && rightSlide.isBusy()) {
//            opMode.telemetry.addData("FR Target", leftSlide.getTargetPosition());
//            opMode.telemetry.addData("FR Position", rightSlide.getCurrentPosition());
//            opMode.telemetry.update();
        }
        leftSlide.setPower(0);
        rightSlide.setPower(0);
    }

    public void heightToFirstBasket(){
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        leftSlide.setTargetPosition(-2000);
        rightSlide.setTargetPosition(-2000);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(0.4);
        rightSlide.setPower(0.4);

        while (leftSlide.isBusy() && rightSlide.isBusy()) {
//            opMode.telemetry.addData("FR Target", leftSlide.getTargetPosition());
//            opMode.telemetry.addData("FR Position", rightSlide.getCurrentPosition());
//            opMode.telemetry.update();
        }
        leftSlide.setPower(0);
        rightSlide.setPower(0);
    }

    public void heightToSecondBasket(){
        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        leftSlide.setTargetPosition(-3700);
        rightSlide.setTargetPosition(-3700);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(0.4);
        rightSlide.setPower(0.4);

        while (leftSlide.isBusy() && rightSlide.isBusy()) {
//            opMode.telemetry.addData("FR Target", leftSlide.getTargetPosition());
//            opMode.telemetry.addData("FR Position", rightSlide.getCurrentPosition());
//            opMode.telemetry.update();
        }
        leftSlide.setPower(0);
        rightSlide.setPower(0);
    }

    public void armUp(){ //moving arm up gradually
        double a = armLeft.getPosition();
        a += 0.001;
        armLeft.setPosition(a);
        armRight.setPosition(a);
        a = armLeft.getPosition();
    }

    public void armDown(){ //moving arm down gradually
        double a = armLeft.getPosition();
        a -= 0.001;
        armLeft.setPosition(a);
        armRight.setPosition(a);
        a = armLeft.getPosition();
    }

    public void slideUp(){ //fine-tuning slide position go up
        int leftPos = leftSlide.getCurrentPosition();
        leftPos -= 100;
        if(leftPos <= -3700){ //can change this value to upper limit
            leftPos = -3700;
        }
        leftSlide.setTargetPosition(leftPos);
        int rightPos = leftSlide.getCurrentPosition();
        rightPos -= 100;
        if(rightPos <= -3700){ //can change this value to upper limit
            rightPos = -3700;
        }
        rightSlide.setTargetPosition(rightPos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(0.7);
        rightSlide.setPower(0.7);
    }

    public void slideDown(){ //fine-tuning slide position go down
        int leftPos = leftSlide.getCurrentPosition();
        leftPos += 100;
        if(leftPos >= -300){
            leftPos = -300;
        }
        leftSlide.setTargetPosition(leftPos);
        int rightPos = leftSlide.getCurrentPosition();
        rightPos += 100;
        if(rightPos >= -300){
            rightPos = -300;
        }
        rightSlide.setTargetPosition(rightPos);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlide.setPower(0.7);
        rightSlide.setPower(0.7);
    }

    public void clawUp(){
        double b = claw.getPosition();
        b += 0.002;
        claw.setPosition(b);
        b = claw.getPosition();
    }

    public void clawDown(){
        double b = claw.getPosition();
        b -= 0.002;
        claw.setPosition(b);
        b = claw.getPosition();
    }

    public void spinToRight(){
        double b = clawRotation.getPosition();
        b += 0.002;
        clawRotation.setPosition(b);
        b = clawRotation.getPosition();
    }

    public void spinToLeft(){
        double b = clawRotation.getPosition();
        b -= 0.002;
        clawRotation.setPosition(b);
        b = clawRotation.getPosition();
    }

    public void clawOpen(){
        clawControl.setPosition(0.65);
    }

    public void clawClose(){
        clawControl.setPosition(0.88);
    }

}
