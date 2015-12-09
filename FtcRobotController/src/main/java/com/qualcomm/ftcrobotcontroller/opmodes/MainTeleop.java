package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by mjensson on 11/5/2015.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class MainTeleop extends OpMode {
    // position of the arm servo
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

    int numOpLoops = 0;

     /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
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
      if(driveController.getMotorControllerDeviceMode() == writeOnly){
          float left = -gamepad1.left_stick_y;
          float right = -gamepad1.right_stick_y;

          left = Range.clip(left, -1, 1);
          right = Range.clip(right, -1, 1);

          left = (float)scaleInput(left);
          right = (float)scaleInput(right);

          driveLeft.setPower(left);
          driveRight.setPower(right);
      }

        if(armController.getMotorControllerDeviceMode() == writeOnly){
            float arm = -gamepad2.left_stick_y;

            arm = Range.clip(arm, -1, 1);

            arm = (float)scaleInput(arm);

            motorArm.setPower(arm);
        }

        if (numOpLoops % 14 == 0){
            driveController.setMotorControllerDeviceMode(readOnly);
            armController.setMotorControllerDeviceMode(readOnly);

            telemetry.addData("left drive power", driveLeft.getPower());
            telemetry.addData("left drive position", driveLeft.getCurrentPosition());

            telemetry.addData("right drive power", driveRight.getPower());
            telemetry.addData("right drive position", driveRight.getCurrentPosition());

            telemetry.addData("arm motor", motorArm.getPower());
            telemetry.addData("arm motor position", motorArm.getCurrentPosition());
            telemetry.addData("servo position", servoArm.getPosition());
            telemetry.addData("RunMode: ", motorArm.getChannelMode().toString());

            // Only needed on Nxt devices, but not on USB devices
            driveController.setMotorControllerDeviceMode(writeOnly);
            armController.setMotorControllerDeviceMode(writeOnly);
            // Reset the loop
            numOpLoops = 0;
        }

        numOpLoops++;
    }

    @Override
    public void stop() {

    }

    /*
 * This method scales the joystick input so for low joystick values, the
 * scaled value is less than linear.  This is to make it easier to drive
 * the robot more precisely at slower speeds.
 */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}
