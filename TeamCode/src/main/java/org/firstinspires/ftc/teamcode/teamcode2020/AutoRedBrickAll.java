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


@TeleOp(name = "AutoRedBrickAll", group = "auto")
//originally had it as TeleOp b/c Autonomous wasn't working, but changed back over
public class AutoRedBrickAll extends LinearOpMode {
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
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensorRight");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensorRight");
        floorColorSensor = hardwareMap.get(ColorSensor.class, "floorColorSensor");

        double distTravelled = 2.0;
        double feederPow = 0;
        double feederWide = 0;
        final double SCALE_FACTOR = 255;
        float hsvValues[] = {0F, 0F, 0F};
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


            //robot starts parallel to wall and moves horizontal to be next to and parallel to the bricks
            //while too far away, move closer
            while (!(distanceSensor.getDistance(DistanceUnit.INCH)<3.6)){
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


            TimeUnit.MILLISECONDS.sleep(800);


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

            //SLIIIIIIIDE to the right
            robot.gyrodrive.horizontal(0.7, Convert.tileToYeetGV(.6), robot.getHeading());



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



            robot.gyrodrive.horizontal(0.7, Convert.tileToYeetGV(-.8), robot.getHeading());


            //go back until blue line
            while (floorColorSensor.red()<1000){ /////TODO FIND A VALUE FOR RED
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


            //turn to platform on right (not moved by alliance partner)
            robot.gyrodrive.turn(0.7, -90);





            //////////////////////////////////////YO TIANA move forward to platform here




            //open feeder to let go of block
            feederWide = 0.5;
            robot.mfeedMotor.setPower(feederWide);
            TimeUnit.MILLISECONDS.sleep(900);
            feederWide = 0;
            robot.mfeedMotor.setPower(feederWide);


            //after drop off block, come out, go around block, and go back to platform
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(-.4), robot.getHeading());
            robot.gyrodrive.horizontal(0.7, Convert.tileToYeetGV(-.6), robot.getHeading());
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(.4), robot.getHeading());


            ///////////////////////////// YO TIANA servo down for platform here

            //drag platform back
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(-1.4), robot.getHeading());

            //make sure facing correct direction
            robot.gyrodrive.turn(0.7, -90);


            ///move left to red line
            while (floorColorSensor.blue()<1000){
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

            break;
        }
    }
}