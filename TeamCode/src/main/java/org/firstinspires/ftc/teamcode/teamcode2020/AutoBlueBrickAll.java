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


@TeleOp(name = "AutoBlueBrickAll", group = "auto")
//originally had it as TeleOp b/c Autonomous wasn't working, but changed back over
public class AutoBlueBrickAll extends LinearOpMode {
    private Robot2017 robot;
    private ElapsedTime runtime = new ElapsedTime();
    private ColorSensor colorSensor;
    private ColorSensor floorColorSensor;
    private TouchSensor touchSensor;
    private DistanceSensor distanceSensor;


    public void runOpMode() throws InterruptedException {
        robot = new Robot2017();
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);
        robot.setTime(runtime);

        //nameOfThingInCode = hardwareMap.typeOfThing.get("nameOfThingInConfiguration");
        touchSensor = hardwareMap.touchSensor.get("touchSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");
        floorColorSensor = hardwareMap.get(ColorSensor.class, "floorColorSensor");


        double distTravelled = 2.0;
        double feederPow = 0;
        double feederWide = 0;
        ///////////////TODO before competition change floorBlue and close feeder
        int floorBlue = 600;
        final double SCALE_FACTOR = 255;
        float hsvValues[] = {0F, 0F, 0F};
        boolean skystone = false;
        int stoneCount = 0;
        double v1 = .4;


        //MAKE SURE FEEDER IS IN CLOSED POSITION BEFORE START

        //inputGameConfig();

        //Wait for the match to begin, press start button
        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        while (opModeIsActive()) {

            //color sensor scale factors
            Color.RGBToHSV((int)(colorSensor.red() * SCALE_FACTOR), (int) (colorSensor.green() * SCALE_FACTOR), (int) (colorSensor.blue() * SCALE_FACTOR), hsvValues);
            Color.RGBToHSV((int)(floorColorSensor.red() * SCALE_FACTOR), (int) (floorColorSensor.green() * SCALE_FACTOR), (int) (floorColorSensor.blue() * SCALE_FACTOR), hsvValues);

            robot.composeIMUTelemetry();


            //robot starts parallel to wall and moves horizontal to be next to and parallel to the bricks
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

            robot.gyrodrive.turn(0.7, 0);


            //while not skystone, move forwards
            while (!(colorSensor.red() + colorSensor.blue() + colorSensor.green() < 700)){
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



            //robot goes back to knock the other bricks out of the way to be in front of the skystone
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(-.75), robot.getHeading());  // .55 is the length of the front of the robot



            //SLIIIIIIIDE to the left
            robot.gyrodrive.horizontal(0.7, Convert.tileToYeetGV(-.55), robot.getHeading());



            //open feeder
            //negative is open
            feederWide = 0.5;
            robot.mfeedMotor.setPower(-feederWide);
            TimeUnit.MILLISECONDS.sleep(1000);
            robot.mfeedMotor.setPower(0);



            //move forward to block
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(.45), robot.getHeading());



            //close feeder
            robot.mfeedMotor.setPower(feederWide);
            TimeUnit.MILLISECONDS.sleep(900);
            robot.mfeedMotor.setPower(0);



            //while block not in feeder, feed
            while (touchSensor.getValue() != 1) {
                feederPow = .5;
                robot.rfeedMotor.setPower(-feederPow);
                robot.lfeedMotor.setPower(feederPow);
            }
            robot.rfeedMotor.setPower(0);
            robot.lfeedMotor.setPower(0);


            robot.gyrodrive.horizontal(0.7, Convert.tileToYeetGV(.9), robot.getHeading());

            ////////////////// TODO comment out below and use touch sensor on back instead?



            //go back until blue line
            while (floorColorSensor.blue()<floorBlue){
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

            TimeUnit.MILLISECONDS.sleep(1000);


            //from middle blue line, move back towards platform
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(-2.8), 0);


            //turn to platform on left (not moved by alliance partner)
            robot.gyrodrive.turn(0.7, 90);



            //lift block over platform
            //so apparently negative is up
            robot.liftMotor.setPower(-.8);
            TimeUnit.MILLISECONDS.sleep(900);
            robot.liftMotor.setPower(0);
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(.2), 0);
            robot.liftMotor.setPower(.2);
            TimeUnit.MILLISECONDS.sleep(500);
            robot.liftMotor.setPower(0);


            //open feeder to let go of block
            robot.mfeedMotor.setPower(-feederWide);
            TimeUnit.MILLISECONDS.sleep(900);
            robot.mfeedMotor.setPower(0);


            //after drop off block, come out, go around block, and go back to platform
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(-.4), robot.getHeading());
            robot.gyrodrive.horizontal(0.7, Convert.tileToYeetGV(.6), robot.getHeading());
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(.4), robot.getHeading());


            ///////////////////////////// TODO find up down angles
            robot.servoDown();


            TimeUnit.MILLISECONDS.sleep(1000);

            //drag platform back
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(-1.4), robot.getHeading());


            ////////////TODO idk bro
            robot.servoUp();


            //make sure facing correct direction
            robot.gyrodrive.turn(0.7, 90);


            ///move right to blue line
            while (floorColorSensor.blue()<floorBlue){
                ///right
                robot.flMotor.setPower(-v1);
                robot.frMotor.setPower(v1);
                robot.blMotor.setPower(v1);
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