package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by mjensson on 11/9/2015.
 */
public class MainAutonomous extends OpMode {
    final static double MOTOR_POWER = 0.15; // Higher values will cause the robot to move faster
    final static double HOLD_IR_SIGNAL_STRENGTH = 0.50; // Higher values will cause the robot to follow closer

    double armPosition;
    double armDelta = 0.1;

    DcMotorController.DeviceMode writeOnly = DcMotorController.DeviceMode.WRITE_ONLY;
    DcMotorController.DeviceMode readOnly = DcMotorController.DeviceMode.READ_ONLY;

    DcMotorController driveController;
    DcMotor driveRight;
    DcMotor driveLeft;

    DcMotorController armController;
    DcMotor motorArm;

    Servo servoArm;

    IrSeekerSensor irSeeker;

    int numOpLoops = 0;

    public MainAutonomous(){

    }

    @Override
    public void init() {
        driveController = hardwareMap.dcMotorController.get("driveMotorController");
        driveRight = hardwareMap.dcMotor.get("driveMotorRight");
        driveLeft = hardwareMap.dcMotor.get("driveMotorLeft");

        armController = hardwareMap.dcMotorController.get("armMotorController");
        motorArm = hardwareMap.dcMotor.get("armMotor");

        servoArm = hardwareMap.servo.get("manipulatorServo"); // channel 6

        driveController.setMotorChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        armController.setMotorChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        irSeeker = hardwareMap.irSeekerSensor.get("ir_seeker");
    }

    @Override
    public void init_loop() {
        driveRight.setDirection(DcMotor.Direction.REVERSE);

        driveLeft.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        driveRight.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorArm.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    @Override
    public void loop() {
        double[] motorValues =

        driveLeft.setPower(left);
        driveRight.setPower(right);
    }

    public double[] setMotorPowerValues(){
        double angle = 0.0;
        double strength = 0.0;
        double left, right = 0.0;
        double[] values = {0.0, 0.0};

        if(irSeeker.signalDetected()){
            angle = irSeeker.getAngle();
            strength = irSeeker.getStrength();
            if( angle < -60){
                left = -MOTOR_POWER;
                right = MOTOR_POWER;
            } else if(angle < -5){
                left = -MOTOR_POWER - 0.05;
                right = MOTOR_POWER;
            } else if(angle > 5 && angle < 60){
                left = MOTOR_POWER;
                right = -MOTOR_POWER - 0.05;
            } else if(angle > 60){
                left = MOTOR_POWER;
                right = -MOTOR_POWER;
            } else if(strength < HOLD_IR_SIGNAL_STRENGTH){
                left = MOTOR_POWER;
                right = MOTOR_POWER;
            } else {
                left = 0.0;
                right = 0.0;
            }
        } else {
            left = 0.0;
            right  0.0;
        }

        values[0] = left;
        values[1] = right;

        return values;
    }
}
