package org.firstinspires.ftc.teamcode.teamcode2020;

/**
 * Created by theCoolestKidEver on 9/13/19.
 */

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.game.robot.Convert;

import java.util.concurrent.TimeUnit;


@TeleOp(name = "AutoRedBrickAll", group = "auto")
//originally had it as TeleOp b/c Autonomous wasn't working, but changed back over
public class AutoRedBrickOtherSide extends LinearOpMode {
    private Robot2017 robot;
    private ElapsedTime runtime = new ElapsedTime();
    private ColorSensor colorSensor;
    private ColorSensor floorColorSensor;
    private TouchSensor touchSensor;
    private DistanceSensor distanceSensor;
    private Servo lServo;
    private Servo rServo;

    public void runOpMode() throws InterruptedException {
        robot = new Robot2017();
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);
        robot.setTime(runtime);

        //nameOfThingInCode = hardwareMap.typeOfThing.get("nameOfThingInConfiguration");
        touchSensor = hardwareMap.touchSensor.get("touchSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensorRight");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensorRight");
        floorColorSensor = hardwareMap.get(ColorSensor.class, "floorColorSensor");
        lServo = hardwareMap.servo.get("lServo");
        rServo = hardwareMap.servo.get("rServo");

        double distTravelled = 2.0;
        double feederPow = 0;
        double feederWide = 0;
        final double SCALE_FACTOR = 255;
        float hsvValues[] = {0F, 0F, 0F};
        int floorBlue = 600;
        boolean skystone = false;
        int stoneCount = 0;
        double v1 = .4;


        //MAKE SURE FEEDER IS IN CLOSED POSITION BEFORE START


        //Wait for the match to begin, press start button
        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        while (opModeIsActive()) {

            //color sensor scale factors
            Color.RGBToHSV((int)(colorSensor.red() * SCALE_FACTOR), (int) (colorSensor.green() * SCALE_FACTOR), (int) (colorSensor.blue() * SCALE_FACTOR), hsvValues);
            Color.RGBToHSV((int)(floorColorSensor.red() * SCALE_FACTOR), (int) (floorColorSensor.green() * SCALE_FACTOR), (int) (floorColorSensor.blue() * SCALE_FACTOR), hsvValues);

            robot.composeIMUTelemetry();


            //robot starts facing platform
            //while too far away, move closer
            /*
            while (!something){
                robot.flMotor.setPower(-v1);
                robot.frMotor.setPower(-v1);
                robot.blMotor.setPower(-v1);
                robot.brMotor.setPower(-v1);
            }
            */
            robot.flMotor.setPower(0);
            robot.frMotor.setPower(0);
            robot.blMotor.setPower(0);
            robot.brMotor.setPower(0);

            //Grab platform
            lServo.setPosition(-.3f);
            rServo.setPosition(-.6f);

            //Drag platform backup
            /*
            while (!something){
                robot.flMotor.setPower(v1);
                robot.frMotor.setPower(v1);
                robot.blMotor.setPower(v1);
                robot.brMotor.setPower(v1);
            }
            */
            robot.flMotor.setPower(0);
            robot.frMotor.setPower(0);
            robot.blMotor.setPower(0);
            robot.brMotor.setPower(0);

            //Release platform
            lServo.setPosition(.2f);
            rServo.setPosition(.6f);

            //Move right towards blocks side
            robot.gyrodrive.horizontal(0.7, Convert.tileToYeetGV(-3.25), robot.getHeading());

            //Turn left
            robot.gyrodrive.turn(0.7, 90);

            //Start Stolen code
            //TODO determine how much time is left

            //while too far away, move closer
            while (!(distanceSensor.getDistance(DistanceUnit.INCH)<3.6)){
                ///left
                robot.flMotor.setPower(v1);
                robot.frMotor.setPower(-v1);
                robot.blMotor.setPower(-v1);
                robot.brMotor.setPower(v1);
            }
            robot.flMotor.setPower(0);
            robot.frMotor.setPower(0);
            robot.blMotor.setPower(0);
            robot.brMotor.setPower(0);


            TimeUnit.MILLISECONDS.sleep(1000);


            //while not skystone, move backwards
            while (!(colorSensor.red() + colorSensor.blue() + colorSensor.green() < 600)){
                ///backward
                robot.flMotor.setPower(v1);
                robot.frMotor.setPower(v1);
                robot.blMotor.setPower(v1);
                robot.brMotor.setPower(v1);
            }
            robot.flMotor.setPower(0);
            robot.frMotor.setPower(0);
            robot.blMotor.setPower(0);
            robot.brMotor.setPower(0);



            //robot goes back to knock the other bricks out of the way to be in front of the skystone
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(-1.05), robot.getHeading());  // .55 is the length of the front of the robot



            //SLIIIIIIIDE to the left
            robot.gyrodrive.horizontal(0.7, Convert.tileToYeetGV(-.6), robot.getHeading());



            //open feeder
            feederWide = 0.5;
            robot.mfeedMotor.setPower(feederWide);
            TimeUnit.MILLISECONDS.sleep(900);
            feederWide = 0;
            robot.mfeedMotor.setPower(feederWide);



            //move forward to block
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(.45), robot.getHeading());



            //close feeder
            feederWide = -0.5;
            robot.mfeedMotor.setPower(feederWide);
            TimeUnit.MILLISECONDS.sleep(900);
            feederWide = 0;
            robot.mfeedMotor.setPower(feederWide);



            //while block not in feeder, feed
            while (touchSensor.getValue() != 1) {
                feederPow = .5;
                robot.rfeedMotor.setPower(-feederPow);
                robot.lfeedMotor.setPower(feederPow);
            }
            feederPow = 0;
            robot.rfeedMotor.setPower(-feederPow);
            robot.lfeedMotor.setPower(feederPow);


            robot.gyrodrive.horizontal(0.7, Convert.tileToYeetGV(.8), robot.getHeading());


            //go forwards until red line
            while (floorColorSensor.red()<700){
                ///forwards
                robot.flMotor.setPower(-v1);
                robot.frMotor.setPower(-v1);
                robot.blMotor.setPower(-v1);
                robot.brMotor.setPower(-v1);
            }
            robot.flMotor.setPower(0);
            robot.frMotor.setPower(0);
            robot.blMotor.setPower(0);
            robot.brMotor.setPower(0);

            TimeUnit.MILLISECONDS.sleep(1000);


            //from middle red line, move forwards towards platform
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(2.3), 0);


            //turn to platform on right (already moved by alliance partner)
            robot.gyrodrive.turn(0.7, -90);



            //open feeder to let go of block
            feederWide = 0.5;
            robot.mfeedMotor.setPower(feederWide);
            TimeUnit.MILLISECONDS.sleep(900);
            feederWide = 0;
            robot.mfeedMotor.setPower(feederWide);


            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(-.6), robot.getHeading());

            ///move back to red line
            robot.gyrodrive.horizontal(0.7, Convert.tileToYeetGV(2.3), robot.getHeading());

            break;
        }
    }
}