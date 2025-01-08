package org.firstinspires.ftc.teamcode.monkeyballs;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Disabled
public class Drivetrain {
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftRear;
    public DcMotor rightRear;

    IMU imu;
    YawPitchRollAngles ypr;

    LinearOpMode opMode;

    public Drivetrain(LinearOpMode opMode, HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu = hardwareMap.get(IMU.class, "imu");
        ypr = imu.getRobotYawPitchRollAngles();
        imu.initialize(parameters);
        imu.resetYaw();
        this.opMode = opMode;

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        startEncoders();

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public void drive(int forward, double power) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(forward);
        rightFront.setTargetPosition(forward);
        leftRear.setTargetPosition(forward);
        rightRear.setTargetPosition(forward);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);

        while (leftFront.isBusy() && rightFront.isBusy()) {
            opMode.telemetry.addData("FR Target", rightFront.getTargetPosition());
            opMode.telemetry.addData("FR Position", rightFront.getCurrentPosition());
            opMode.telemetry.addData("leftFront Target", leftFront.getTargetPosition());
            opMode.telemetry.addData("leftFront Position", leftFront.getCurrentPosition());
            opMode.telemetry.update();
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public void strafe(int amount, double power, String direction) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (direction.equals("right")) {
            leftFront.setTargetPosition(amount);
            rightFront.setTargetPosition(-amount);
            leftRear.setTargetPosition(-amount);
            rightRear.setTargetPosition(amount);
        } else if (direction.equals("left")) {
            leftFront.setTargetPosition(-amount);
            rightFront.setTargetPosition(amount);
            leftRear.setTargetPosition(amount);
            rightRear.setTargetPosition(-amount);
        }

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);

        while (leftFront.isBusy() || rightFront.isBusy()) {
            opMode.telemetry.addData("FR Target", rightFront.getTargetPosition());
            opMode.telemetry.addData("FR Position", rightFront.getCurrentPosition());
            opMode.telemetry.addData("leftFront Target", leftFront.getTargetPosition());
            opMode.telemetry.addData("leftFront Position", leftFront.getCurrentPosition());
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public void turn(double amount, double power, String direction) {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu.resetYaw();
        ypr = imu.getRobotYawPitchRollAngles();

        if (direction.equals("left")) power *= -1;

        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftRear.setPower(power);
        rightRear.setPower(-power);

        while (opMode.opModeIsActive() && Math.abs(ypr.getYaw(AngleUnit.DEGREES)) < amount) {
            opMode.telemetry.addData("Current Angle", ypr.getYaw(AngleUnit.DEGREES));
            opMode.telemetry.addData("Target Angle", amount);
            opMode.telemetry.addData("Direction", direction);
            opMode.telemetry.update();

            ypr = imu.getRobotYawPitchRollAngles(); //keeps updating ypr until ideal angle is reached
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        startEncoders();
    }

    private void startEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double getYaw() {
        ypr = imu.getRobotYawPitchRollAngles();
        return ypr.getYaw(AngleUnit.DEGREES);
    }

    public void piddrive(int forward) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double kp = 1 / 300.0; //constant value that is multiplied by error to find power

        leftFront.setTargetPosition(forward);
        rightFront.setTargetPosition(forward);
        leftRear.setTargetPosition(forward);
        rightRear.setTargetPosition(forward);

        double error1 = forward - leftFront.getCurrentPosition();
        double error2 = forward - rightFront.getCurrentPosition();

        while (Math.abs(error1) > 15 && Math.abs(error2) > 15) {
            error1 = forward - leftFront.getCurrentPosition();
            error2 = forward - rightFront.getCurrentPosition();
            leftFront.setPower(kp * (forward - leftFront.getCurrentPosition()));
            rightFront.setPower(kp * (forward - rightFront.getCurrentPosition()));
            leftRear.setPower(kp * (forward - leftRear.getCurrentPosition()));
            rightRear.setPower(kp * (forward - rightRear.getCurrentPosition()));
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public void piddrive(int forward, double kpgiven) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double kp = kpgiven;

        leftFront.setTargetPosition(forward);
        rightFront.setTargetPosition(forward);
        leftRear.setTargetPosition(forward);
        rightRear.setTargetPosition(forward);

        double error1 = forward - leftFront.getCurrentPosition();
        double error2 = forward - rightFront.getCurrentPosition();

        while (Math.abs(error1) > 15 && Math.abs(error2) > 15) {
            error1 = forward - leftFront.getCurrentPosition();
            error2 = forward - rightFront.getCurrentPosition();
            leftFront.setPower(kp * (forward - leftFront.getCurrentPosition()));
            rightFront.setPower(kp * (forward - rightFront.getCurrentPosition()));
            leftRear.setPower(kp * (forward - leftRear.getCurrentPosition()));
            rightRear.setPower(kp * (forward - rightRear.getCurrentPosition()));
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public void pidstrafe(int amount, String direction) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double kp = 1 / 300.0;

        if (direction.equals("right")) {
            leftFront.setTargetPosition(amount);
            rightFront.setTargetPosition(-amount);
            leftRear.setTargetPosition(-amount);
            rightRear.setTargetPosition(amount);
        } else if (direction.equals("left")) {
            leftFront.setTargetPosition(-amount);
            rightFront.setTargetPosition(amount);
            leftRear.setTargetPosition(amount);
            rightRear.setTargetPosition(-amount);
        }

        double error1 = leftFront.getTargetPosition() - leftFront.getCurrentPosition();
        double error2 = rightFront.getTargetPosition() - rightFront.getCurrentPosition();

        while (Math.abs(error1) > 15 && Math.abs(error2) > 15) {
            error1 = leftFront.getTargetPosition() - leftFront.getCurrentPosition();
            error2 = rightFront.getTargetPosition() - rightFront.getCurrentPosition();
            leftFront.setPower(kp * (leftFront.getTargetPosition() - leftFront.getCurrentPosition()));
            rightFront.setPower(kp * (rightFront.getTargetPosition() - rightFront.getCurrentPosition()));
            leftRear.setPower(kp * (leftRear.getTargetPosition() - leftRear.getCurrentPosition()));
            rightRear.setPower(kp * (rightRear.getTargetPosition() - rightRear.getCurrentPosition()));
        }

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    public void wait(int ms) {
        ElapsedTime t = new ElapsedTime();
        t.startTime();
        while (t.milliseconds() < ms) {
        }
        t.reset();
    }

    /*public boolean sampleDetection(String alliance) {
        double r = colorSensor.red();
        double g = colorSensor.green();
        double b = colorSensor.blue();

        if (alliance.equals("red") && ((r > g && r > b && r > 300) || (g > r && g > b && g > 300))) {
            return true;
        } else if (alliance.equals("blue") && ((b > r && b > g && b > 300) || (g > r && g > b && g > 300))) {
            return true;
        }
        return false;
    }

    public void samplePickerUpper(String alliance){
        boolean doWeWantIt = sampleDetection(alliance);
        if(doWeWantIt){
            intakeLeft.setPower(0.6);
            intakeRight.setPower(0.6);
        }
        else{
            intakeLeft.setPower(-0.6);
            intakeRight.setPower(-0.6);
        }
    }
    */

}