/**
 * Created by neeraja on 11/6/2016.
 */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Teleop Boat", group="Pushbot")
public class TeleOp extends OpMode {

    DcMotor motorRight;
    DcMotor motorLeft;
// Servo gripperRight;
//   Servo gripperLeft;
//    DcMotor motorElevator;
//    Servo servoRightArm;

    /**
     * Constructor
     */
    public TeleOp() {



    }

    /*
     * Code that runs when the op mode is first enabled.
     */
    @Override
    public void init() {

        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
//      gripperRight = hardwareMap.servo.get("gripperRight");//
//      gripperLeft = hardwareMap.servo.get("gripperLeft");//
//        motorElevator = hardwareMap.dcMotor.get("motorElevator");
//        servoRightArm = hardwareMap.servo.get("servoRightArm");//


        //port 1 is gripperleft
        //port 2 is gripperRight


    }

    /*
     * This method will be called repeatedly in a loop
     */
    @Override
    public void loop() {

		/*
		 * Gamepad 1 controls the drive motors.
		 */

        // tank drive
        // y equals -1 means the joystick is pushed all of the way forward.
        //x equals -1 means the joystick 0is pushed all the way to the left.

//        float elevator = -gamepad2.right_stick_y;
//        float launcher = gamepad2.left_stick_y;
//        float throttle = -gamepad1.left_stick_x;




        float throttle = -gamepad1.right_stick_x;
        float direction = -gamepad1.left_stick_y;
//        float elevatorthrottle = -gamepad2.right_stick_y;
//        float elevatorthrottleReverse = gamepad2.left_stick_y;
        float right = throttle - direction;
        float left = (throttle + direction);


//        gripperRight.scaleRange();


//        if (gamepad2.dpad_down) {
//           gripperRight.setPosition(1);
//          gripperLeft.setPosition(0);
//
//        } else if (gamepad2.dpad_up) {
//           gripperRight.setPosition(0);
//            gripperLeft.setPosition(1);
//
//        }







        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

//        brush = Range.clip(brush, -1, 1);
//        elevator = Range.clip(elevator, -1, 1);
//        launcher = Range.clip(launcher, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float) scaleInput(right);
        left = (float) scaleInput(left);
//        brush = (float)scaleInput(brush);
//        elevator = (float)scaleInput(elevator);
//        launcher = (float)scaleInput(launcher);

        // write the values to the motors
//        motorRight.setPower(0.6*right);
        motorRight.setPower(1 * right);

//        motorLeft.setPower(0.6*left);
        motorLeft.setPower(1 * left);

//        motorElevator.setPower(.50 * elevatorthrottle);
//        motorElevator.setPower(.50 * elevatorthrottleReverse);






        telemetry.addData("left power", left);
        telemetry.addData("right power", right);
    }




       // telemetry.addData("elevator power", elevatorthrottle);
       // telemetry.addData("servo power", servopower);


//        motorLaunchL.setPower(launcher);
//        motorLaunchR.setPower(-launcher);
//        motorBrush.setPower(brush);
//        motorElevator.setPower(elevator);


		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 * *telemetry.addData("Text", "*** Robot Data***");
		 * *telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
		 * *telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
		 */





    /*
     * Code that runs when the op mode is first disabled.
     */
    @Override
    public void stop() {

    }

//    public void driveForward() throws InterruptedException {
//        if(gamepad2.a) {
//            motorElevator.setPower(0.5);
//            sleep(2000);
//
//        }
////        motorRight.setPower(Range.clip(power, -1.0, 1.0));
//    }

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
