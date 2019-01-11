package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.os.CountDownTimer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.MyService;

import java.sql.Timestamp;
import java.util.Date;

import static java.lang.Thread.sleep;

@Autonomous public class AndroidLocationFinal2LinearOpModeVersion extends LinearOpMode implements SensorEventListener {

    private static Location initialLocation;
    private static long timeSinceLastLocationUpdateInMillis;
    private static final long START_TIME_IN_MILLISECONDS = 600000;
    private CountDownTimer mCountDownTimer;
    private boolean mTimerRunning;
    private long mTimeLeftinMillis = START_TIME_IN_MILLISECONDS;
//    private String startDate;
    private SensorManager mSensorManager;
//    private Sensor mCompass;
    private Sensor accelerometer;
    private Sensor magnetometer;
    DcMotor motorRight;
    DcMotor motorLeft;
    private static Location currentLocation = new Location("");
    private static Location targetLocation = new Location("");
    private static double[] targetDisplacementVector = new double[2];
    private static double[] actualDisplacementVector = new double[2];
    public static int iterations = 0;



    // orientation values
    private static double azimuth = 0.0f;      // value in radians
    private static float pitch = 0.0f;        // value in radians
    private static float roll = 0.0f;         // value in radians

    private float[] mGravity;       // latest sensor values
    private float[] mGeomagnetic;   // latest sensor values

    private static double targetAzimuth;
    private static Timestamp storedTimeStamp = null;


    @Override
    public void runOpMode() throws InterruptedException {

        mSensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
//        mCompass = mSensorManager.getDefaultSensor(Sensor.TYPE_ORIENTATION);
        accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");

        azimuth = 0.0f;      // value in radians
        pitch = 0.0f;        // value in radians
        roll = 0.0f;

        targetLocation.setLatitude(40.13343);
        targetLocation.setLongitude(-78);

        currentLocation.setLatitude(MyService.getLocation().getLatitude());
        currentLocation.setLongitude(MyService.getLocation().getLongitude());

        initialLocation = currentLocation;

        targetDisplacementVector[0] = (targetLocation.getLatitude()) - (currentLocation.getLatitude());
        targetDisplacementVector[1] = (targetLocation.getLongitude()) - (currentLocation.getLatitude());

        double hypotenuse = Math.sqrt(targetDisplacementVector[0]*targetDisplacementVector[0] + targetDisplacementVector[1]*targetDisplacementVector[1]);
        double currentTheta = Math.asin(targetDisplacementVector[0]/hypotenuse);
        currentTheta = (Math.PI/2) - currentTheta;

        if(targetDisplacementVector[0] < 0) {
            currentTheta = - currentTheta;
        }

        targetAzimuth = currentTheta;

        waitForStart();

        currentLocation = MyService.getLocation();
        startTimer();
        mSensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_UI);
        mSensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_UI);

           turnByAzimuth(targetAzimuth);
           moveRobot(0.25);

        while(opModeIsActive()) {

            if(MyService.getLocation() != null) {
                telemetry.addData("Integer Test", MyService.lastInt);
                if(currentLocation != MyService.getLocation()) {

                    actualDisplacementVector[0] = MyService.getLocation().getLatitude() - initialLocation.getLatitude();
                    actualDisplacementVector[1] = MyService.getLocation().getLongitude() - initialLocation.getLongitude();

                    double dDOTa = (targetDisplacementVector[0]*actualDisplacementVector[0] + targetDisplacementVector[1]*actualDisplacementVector[1]);
                    double aCROSSd = -(targetDisplacementVector[0]*actualDisplacementVector[1] - targetDisplacementVector[1]*actualDisplacementVector[0]);
                    double MagnitudeProduct = (Math.sqrt(targetDisplacementVector[0]*targetDisplacementVector[0] + targetDisplacementVector[1]*targetDisplacementVector[1]) )*(Math.sqrt(actualDisplacementVector[0]*actualDisplacementVector[0] + actualDisplacementVector[1]*actualDisplacementVector[1])) ;
                    double cosTheta = dDOTa/MagnitudeProduct;
                    double sinTheta = Math.sqrt(1 - (cosTheta)*(cosTheta));
                    double distanceToPath = sinTheta*Math.sqrt(actualDisplacementVector[0]*actualDisplacementVector[0]+actualDisplacementVector[1]*actualDisplacementVector[1]);

                    distanceToPath = distanceToPath*aCROSSd/Math.abs(aCROSSd);


                    double[] velocityVector = new double[2];

                    startTimer();

                    velocityVector[0] = (MyService.getLocation().getLatitude() - currentLocation.getLatitude())/(timeSinceLastLocationUpdateInMillis);
                    velocityVector[1] = (MyService.getLocation().getLongitude() - currentLocation.getLongitude())/(timeSinceLastLocationUpdateInMillis);

                    double dDOTv = (targetDisplacementVector[0]*velocityVector[0] + targetDisplacementVector[1]*velocityVector[1]);
                    double vCROSSd = -(targetDisplacementVector[0]*velocityVector[1] - targetDisplacementVector[1]*velocityVector[0]);
                    MagnitudeProduct = (Math.sqrt(targetDisplacementVector[0]*targetDisplacementVector[0] + targetDisplacementVector[1]*targetDisplacementVector[1]) )*(Math.sqrt(velocityVector[0]*velocityVector[0] + velocityVector[1]*velocityVector[1])) ;
                    cosTheta = dDOTv/MagnitudeProduct;
                    sinTheta = Math.sqrt(1 - (cosTheta)*(cosTheta));
                    double orthogonalVelocity = sinTheta*Math.sqrt(velocityVector[0]*velocityVector[0]+velocityVector[1]*velocityVector[1]);

                    orthogonalVelocity = orthogonalVelocity*vCROSSd/Math.abs(vCROSSd);

                    turnByPD(distanceToPath, orthogonalVelocity);

                }

                telemetry.addData("Current Latitude", MyService.getLocation().getLatitude());
                telemetry.addData("Current Longitude", MyService.getLocation().getLongitude());

            }

            telemetry.addData("Current Azimuth",(azimuth));
            telemetry.addData("Current Pitch", (pitch));
            telemetry.addData("Current Roll", (roll));

            telemetry.addData("Left motor power", motorLeft.getPower());
            telemetry.addData("Right motor power", motorRight.getPower());

            telemetry.update();
        }

        mSensorManager.unregisterListener(this);

    }



//    @Override
//    public void init() {
//
//        mSensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
////        mCompass = mSensorManager.getDefaultSensor(Sensor.TYPE_ORIENTATION);
//        accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
//        magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
//        motorRight = hardwareMap.dcMotor.get("motorRight");
//        motorLeft = hardwareMap.dcMotor.get("motorLeft");
//
//        azimuth = 0.0f;      // value in radians
//        pitch = 0.0f;        // value in radians
//        roll = 0.0f;
//
//        targetLocation.setLatitude(40.13343);
//        targetLocation.setLongitude(-78);
//
//        currentLocation.setLatitude(MyService.getLocation().getLatitude());
//        currentLocation.setLongitude(MyService.getLocation().getLongitude());
//
//        initialLocation = currentLocation;
//
//        targetDisplacementVector[0] = (targetLocation.getLatitude()) - (currentLocation.getLatitude());
//        targetDisplacementVector[1] = (targetLocation.getLongitude()) - (currentLocation.getLatitude());
//
//        double hypotenuse = Math.sqrt(targetDisplacementVector[0]*targetDisplacementVector[0] + targetDisplacementVector[1]*targetDisplacementVector[1]);
//        double currentTheta = Math.asin(targetDisplacementVector[0]/hypotenuse);
//        currentTheta = (Math.PI/2) - currentTheta;
//
//        if(targetDisplacementVector[0] < 0) {
//            currentTheta = - currentTheta;
//        }
//
//        targetAzimuth = currentTheta;
//
//    }


//    @Override
//    public void start() {
//        currentLocation = MyService.getLocation();
//        startTimer();
//        mSensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_UI);
//        mSensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_UI);
////
////        turnByAzimuth(targetAzimuth);
////        moveRobot(0.25);
//
//
//
//    }




//    @Override
//    public void loop() {
////first action done
////        if(iterations == 0) {
////            turnByAzimuth(targetAzimuth);
////            moveRobot(0.25);
////            iterations++;
////        }
//
//
//        if(MyService.getLocation() != null) {
//            telemetry.addData("Integer Test", MyService.lastInt);
//            if(currentLocation != MyService.getLocation()) {
//
//              actualDisplacementVector[0] = MyService.getLocation().getLatitude() - initialLocation.getLatitude();
//              actualDisplacementVector[1] = MyService.getLocation().getLongitude() - initialLocation.getLongitude();
//
//              double dDOTa = (targetDisplacementVector[0]*actualDisplacementVector[0] + targetDisplacementVector[1]*actualDisplacementVector[1]);
//              double aCROSSd = -(targetDisplacementVector[0]*actualDisplacementVector[1] - targetDisplacementVector[1]*actualDisplacementVector[0]);
//              double MagnitudeProduct = (Math.sqrt(targetDisplacementVector[0]*targetDisplacementVector[0] + targetDisplacementVector[1]*targetDisplacementVector[1]) )*(Math.sqrt(actualDisplacementVector[0]*actualDisplacementVector[0] + actualDisplacementVector[1]*actualDisplacementVector[1])) ;
//              double cosTheta = dDOTa/MagnitudeProduct;
//              double sinTheta = Math.sqrt(1 - (cosTheta)*(cosTheta));
//              double distanceToPath = sinTheta*Math.sqrt(actualDisplacementVector[0]*actualDisplacementVector[0]+actualDisplacementVector[1]*actualDisplacementVector[1]);
//
//              distanceToPath = distanceToPath*aCROSSd/Math.abs(aCROSSd);
//
//
//              double[] velocityVector = new double[2];
//
//                startTimer();
//
//                velocityVector[0] = (MyService.getLocation().getLatitude() - currentLocation.getLatitude())/(timeSinceLastLocationUpdateInMillis);
//              velocityVector[1] = (MyService.getLocation().getLongitude() - currentLocation.getLongitude())/(timeSinceLastLocationUpdateInMillis);
//
//              double dDOTv = (targetDisplacementVector[0]*velocityVector[0] + targetDisplacementVector[1]*velocityVector[1]);
//              double vCROSSd = -(targetDisplacementVector[0]*velocityVector[1] - targetDisplacementVector[1]*velocityVector[0]);
//              MagnitudeProduct = (Math.sqrt(targetDisplacementVector[0]*targetDisplacementVector[0] + targetDisplacementVector[1]*targetDisplacementVector[1]) )*(Math.sqrt(velocityVector[0]*velocityVector[0] + velocityVector[1]*velocityVector[1])) ;
//              cosTheta = dDOTv/MagnitudeProduct;
//              sinTheta = Math.sqrt(1 - (cosTheta)*(cosTheta));
//              double orthogonalVelocity = sinTheta*Math.sqrt(velocityVector[0]*velocityVector[0]+velocityVector[1]*velocityVector[1]);
//
//              orthogonalVelocity = orthogonalVelocity*vCROSSd/Math.abs(vCROSSd);
//
//              turnByPD(distanceToPath, orthogonalVelocity);
//
//            }
//
//            telemetry.addData("Current Latitude", MyService.getLocation().getLatitude());
//            telemetry.addData("Current Longitude", MyService.getLocation().getLongitude());
//
//        }
//
//        telemetry.addData("Current Azimuth",(azimuth));
//        telemetry.addData("Current Pitch", (pitch));
//        telemetry.addData("Current Roll", (roll));
//
//        telemetry.addData("Left motor power", motorLeft.getPower());
//        telemetry.addData("Right motor power", motorRight.getPower());
//
//    }

//    @Override
//    public void stop() {
//        mSensorManager.unregisterListener(this);
//
//    }

    public void onSensorChanged(SensorEvent event) {

//        azimuth = Math.round(event.values[0]);

        // we need both sensor values to calculate orientation
        // only one value will have changed when this method called, we assume we can still use the other value.
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            mGravity = event.values;
        }
        if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            mGeomagnetic = event.values;
        }
        if (mGravity != null && mGeomagnetic != null) {  //make sure we have both before calling getRotationMatrix
            float R[] = new float[9];
            float I[] = new float[9];
            boolean success = SensorManager.getRotationMatrix(R, I, mGravity, mGeomagnetic);
            if (success) {
                float orientation[] = new float[3];
                SensorManager.getOrientation(R, orientation);
                azimuth = orientation[0]; // orientation contains: azimuth, pitch and roll
                pitch = orientation[1];
                roll = orientation[2];
//                telemetry.addData("azimuth", Math.round(Math.toDegrees(azimuth)));
//                telemetry.addData("pitch", Math.round(Math.toDegrees(pitch)));
//                telemetry.addData("roll", Math.round(Math.toDegrees(roll)));
            }

        } else {
            if (mGravity != null) {
                telemetry.addData("note1", "no default accelerometer sensor on phone");
            }
            if (mGeomagnetic != null) {
                telemetry.addData("note2", "no default magnetometer sensor on phone");
            }
        }

//        if (mGravity != null && mGeomagnetic != null) {
//            telemetry.addData("azimuth", Math.round(Math.toDegrees(azimuth)));
//            telemetry.addData("pitch", Math.round(Math.toDegrees(pitch)));
//            telemetry.addData("roll", Math.round(Math.toDegrees(roll)));
//        }
//        else {
//            if (mGravity != null) {
//                telemetry.addData("note1", "no default accelerometer sensor on phone");
//            }
//            if (mGeomagnetic != null) {
//                telemetry.addData("note2", "no default magnetometer sensor on phone");
//            }
//        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }

    public void turnByAzimuth(double targetAzimuth)  {

        double initialPower = motorLeft.getPower();

        while(Math.abs(targetAzimuth - azimuth) >= Math.PI/36 && opModeIsActive()) {

            double right = 0.1;
            double left = 0;
            right = Range.clip(right, -1, 1);
            left = Range.clip(left, -1, 1);

            motorLeft.setPower(left);
            motorRight.setPower(right);


        }

        motorLeft.setPower(0);
        motorRight.setPower(0);

        sleep(1000);

        motorLeft.setPower(initialPower);
        motorRight.setPower(initialPower);


    }

    public void moveRobot( double power) {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

//    private void startTimer() {
//        mCountDownTimer = new CountDownTimer(mTimeLeftinMillis, 1000) {
//            @Override
//            public void onTick(long millisUntilFinished) {
//                mTimeLeftinMillis = millisUntilFinished;
//                timeSinceLastLocationUpdateInMillis  = START_TIME_IN_MILLISECONDS - mTimeLeftinMillis;
//            }
//
//            @Override
//            public void onFinish() {
//
//            }
//        }.start();
//    }

    public void startTimer()  {
        long storedTime = 0;
        if(storedTimeStamp == null) {
            Date date= new Date();
             storedTime = date.getTime();
            storedTimeStamp = new Timestamp(storedTime);
            timeSinceLastLocationUpdateInMillis = 0;
        }

        Date date = new Date();
        long currentTime = date.getTime();
        Timestamp currentTimeStamp = new Timestamp(currentTime);

         timeSinceLastLocationUpdateInMillis = currentTime - storedTime;
        timeSinceLastLocationUpdateInMillis = timeSinceLastLocationUpdateInMillis/1000;

    }

    private void turnByPD(double distance, double velocity) {

        double P = 0;
        double D = 0;
        double total = P*distance + D*velocity;
//        Can put coefficients in front of it - Assume PD added coefficients make total an angle change from -45 to 45

          if(total > 0) {
              double desiredAzimuth = azimuth - total;
              if(desiredAzimuth < -Math.PI) {
                  desiredAzimuth = desiredAzimuth + 2*Math.PI;
              }
              turnByAzimuth(desiredAzimuth);
          }

          else  {
              double desiredAzimuth = azimuth + total;
              if(desiredAzimuth > Math.PI) {
                  desiredAzimuth = desiredAzimuth - 2*Math.PI;
              }
              turnByAzimuth(desiredAzimuth);
          }

    }


}
