package org.firstinspires.ftc.teamcode.teamcode2020;

/**
 * Created by theCoolestKidEver on 9/13/19.
 */

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.game.robot.Convert;
import com.qualcomm.ftccommon.SoundPlayer;
import org.firstinspires.ftc.teamcode.game.robot.StartPosition;

import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "AutoBlueBrickMoved", group = "auto")
//originally had it as TeleOp b/c Autonomous wasn't working, but changed back over
public class AutoBlueBrickMoved extends LinearOpMode {
    private Robot2017 robot;
    private ElapsedTime runtime = new ElapsedTime();
    private ColorSensor colorSensor;
    private ColorSensor floorColorSensor;
    private TouchSensor touchSensor;
    private TouchSensor backTouch;
    private DistanceSensor distanceSensor;
    private TouchSensor servoTouchSensor;
    private Servo lServo;
    private Servo rServo;

    public void servoDown() {
        lServo.setPosition(0.75f);
        rServo.setPosition(0.25f);
        lServo.setPosition(1.0f);
        rServo.setPosition(0.0f);
    }

    public void servoUp() {
        lServo.setPosition(0.75f);
        rServo.setPosition(0.25f);
        lServo.setPosition(0.5f);
        rServo.setPosition(0.5f);
    }


    public void runOpMode() throws InterruptedException {
        robot = new Robot2017();
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);
        robot.setTime(runtime);

        //nameOfThingInCode = hardwareMap.typeOfThing.get("nameOfThingInConfiguration");
        touchSensor = hardwareMap.touchSensor.get("touchSensor");
        backTouch = hardwareMap.touchSensor.get("backTouch");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
        floorColorSensor = hardwareMap.get(ColorSensor.class, "floorColorSensor");
        servoTouchSensor = hardwareMap.touchSensor.get("servoTouchSensor");
        lServo = hardwareMap.servo.get("lServo");
        rServo = hardwareMap.servo.get("rServo");

        double distTravelled = 2.0;
        double feederPow = 0;
        double feederWide = 0;
        int floorBlue = 350;
        final double SCALE_FACTOR = 255;
        float hsvValues[] = {0F, 0F, 0F};
        boolean skystone = false;
        int stoneCount = 0;
        double v1 = .4;
        int floorGrey;
        boolean backWall = false;
        int firstStone;


        //inputGameConfig();

        //MAKE SURE FEEDER IS IN CLOSED POSITION BEFORE START


        //Wait for the match to begin, presses start button
        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        while (opModeIsActive()) {

            Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR), (int) (colorSensor.green() * SCALE_FACTOR), (int) (colorSensor.blue() * SCALE_FACTOR), hsvValues);
            Color.RGBToHSV((int) (floorColorSensor.red() * SCALE_FACTOR), (int) (floorColorSensor.green() * SCALE_FACTOR), (int) (floorColorSensor.blue() * SCALE_FACTOR), hsvValues);

            robot.composeIMUTelemetry();

            //robot starts parallel to wall and moves horizontal to be next to and parallel to the bricks
            //while too far away, move closer
            floorGrey = floorColorSensor.blue();

            while (!(distanceSensor.getDistance(DistanceUnit.INCH) < 3.6)) {
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


            //TimeUnit.MILLISECONDS.sleep(1000);

            robot.gyrodrive.turn(0.7, 0);

            firstStone = colorSensor.red() + colorSensor.blue() + colorSensor.green();


            /*
            //while not skystone, move forwards
            while (!(colorSensor.red() + colorSensor.blue() + colorSensor.green() < 400)) {
                ///forward
                robot.flMotor.setPower(-v1);
                robot.frMotor.setPower(-v1);
                robot.blMotor.setPower(-v1);
                robot.brMotor.setPower(-v1);
            }
            robot.flMotor.setPower(0);
            robot.frMotor.setPower(0);
            robot.blMotor.setPower(0);
            robot.brMotor.setPower(0);
             */

            //TODO YOYO TIANA FIND THE POWER FOR LIFT AND CHANGE FOR ALL OP MODES





            //while not skystone, move forwards
            while (((colorSensor.red() + colorSensor.blue() + colorSensor.green()-firstStone) <240) & ((colorSensor.red() + colorSensor.blue() + colorSensor.green()-firstStone) > -240)) {
                ///forward
                robot.gyrodrive.vertical(0.7, Convert.inchesToYeetGV(7.8), 0);
            }





            //robot goes back to knock the other bricks out of the way to be in front of the skystone
            if ((colorSensor.red() + colorSensor.blue() + colorSensor.green()-firstStone) <240) {
                robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(-.75), robot.getHeading());

            } else {
                robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(-1.0), robot.getHeading());
            }

            // .55 is the length of the front of the robot


            //SLIIIIIIIDE to the left
            robot.gyrodrive.horizontal(0.7, Convert.tileToYeetGV(-.56), 0);


            /*
            //open feeder
            feederWide = 0.5;
            robot.mfeedMotor.setPower(-feederWide);
            TimeUnit.MILLISECONDS.sleep(1000);
            robot.mfeedMotor.setPower(0);

             */

            feederWide = 0.5;


            //move forward to block
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(.45), robot.getHeading());


            //close feeder
            robot.mfeedMotor.setPower(feederWide);
            TimeUnit.MILLISECONDS.sleep(1250);
            robot.mfeedMotor.setPower(0);


            //while block not in feeder, feed
            while (touchSensor.getValue() != 1) {
                feederPow = .5;
                robot.rfeedMotor.setPower(-feederPow);
                robot.lfeedMotor.setPower(feederPow);
            }
            robot.rfeedMotor.setPower(0);
            robot.lfeedMotor.setPower(0);

            //close feeder
            robot.mfeedMotor.setPower(feederWide);
            TimeUnit.MILLISECONDS.sleep(100);
            robot.mfeedMotor.setPower(0);


            robot.gyrodrive.horizontal(0.7, Convert.tileToYeetGV(.9), robot.getHeading());


            //go back until blue line

            while (floorColorSensor.blue() < floorBlue & (floorColorSensor.blue()-floorGrey) < 70 & backTouch.getValue() !=1) {

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

            TimeUnit.MILLISECONDS.sleep(500);

            //from middle blue line, move back towards platform
            if (backTouch.getValue() !=1) {
                robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(-1.3), 0);

                robot.gyrodrive.horizontal(0.7, Convert.tileToYeetGV(-.9), 0);

                robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(-1.5), 0);
            }


            //turn to platform on right (already moved by alliance partner)
            robot.gyrodrive.turn(0.7, -90);


            //lift feeder+block over platform
            robot.liftMotor.setPower(-1);
            TimeUnit.MILLISECONDS.sleep(1800);
            robot.liftMotor.setPower(0);


            //not touching platform, move forward

            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(.5), -90);

            servoUp();


            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(.5), -90);
            //lower feeder
            robot.liftMotor.setPower(.2);
            TimeUnit.MILLISECONDS.sleep(700);
            robot.liftMotor.setPower(0);


            //open feeder to let go of block
            feederWide = 0.5;
            robot.rfeedMotor.setPower(feederPow);
            robot.lfeedMotor.setPower(-feederPow);
            TimeUnit.MILLISECONDS.sleep(600);
            robot.lfeedMotor.setPower(0);
            robot.rfeedMotor.setPower(0);

            robot.mfeedMotor.setPower(-feederWide);
            TimeUnit.MILLISECONDS.sleep(900);
            robot.mfeedMotor.setPower(0);


            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(-.6), -90);
            robot.gyrodrive.horizontal(0.7, Convert.tileToYeetGV(-.6), -90);
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(-.4), -90);

            //make sure facing correct direction
            robot.gyrodrive.turn(0.7, 0);

            // go to the bridge side of the parking
            // robot.gyrodrive.horizontal(0.7, Convert.tileToYeetGV(-.6), 0);

            robot.gyrodrive.vertical(1.0, Convert.tileToYeetGV(2), 0);


            ///move left to blue line
            while (floorColorSensor.blue() < floorBlue) {
                ///left
                robot.flMotor.setPower(-v1);
                robot.frMotor.setPower(-v1);
                robot.blMotor.setPower(-v1);
                robot.brMotor.setPower(-v1);
            }
            robot.flMotor.setPower(0);
            robot.frMotor.setPower(0);
            robot.blMotor.setPower(0);
            robot.brMotor.setPower(0);

            break;

        }
    }
}