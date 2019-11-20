package org.firstinspires.ftc.teamcode.teamcode2020;

/**
 * Created by 22tjiang on 9/13/19.
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


@TeleOp(name = "AutoBlueBrick", group = "auto")
//originally had it as TeleOp b/c Autonomous wasn't working, but changed back over
public class AutoBlueBrick extends LinearOpMode {
    private Robot2017 robot;
    private ElapsedTime runtime = new ElapsedTime();
    private ColorSensor colorSensor;
    private TouchSensor touchSensor;
    private DistanceSensor distanceSensor;

    public void runOpMode() throws InterruptedException {
        robot = new Robot2017();
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);
        robot.setTime(runtime);
        touchSensor = hardwareMap.touchSensor.get("touchSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");

        double distTravelled = 2.0;
        double feederPow = 0;
        double feederWide = 0;
        final double SCALE_FACTOR = 255;
        float hsvValues[] = {0F, 0F, 0F};
        boolean skystone = false;
        int stoneCount = 0;
        double v1 = .4;


        //inputGameConfig();


        //Wait for the match to begin, presses start button
        waitForStart();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        while (opModeIsActive()) {

            Color.RGBToHSV((int)(colorSensor.red() * SCALE_FACTOR), (int) (colorSensor.green() * SCALE_FACTOR), (int) (colorSensor.blue() * SCALE_FACTOR), hsvValues);


            robot.composeIMUTelemetry();

            //robot starts parallel to wall and moves horizontal to be next to and parallel to the bricks


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



            while (!(colorSensor.red() + colorSensor.blue() + colorSensor.green() < 425)){
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
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(-.6), robot.getHeading());  // .55 is the length of the front of the robot
            robot.gyrodrive.horizontal(0.7, Convert.tileToYeetGV(-.6), robot.getHeading());
            robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(.2), robot.getHeading());

            //close feeder
            feederWide = -0.5;
            robot.mfeedMotor.setPower(feederWide);
            TimeUnit.MILLISECONDS.sleep(1000);
            feederWide = 0;
            robot.mfeedMotor.setPower(feederWide);



            while (touchSensor.getValue() != 1) {
                robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(.3), robot.getHeading());
                feederPow = .5;
                robot.rfeedMotor.setPower(-feederPow);
                robot.lfeedMotor.setPower(feederPow);
            }

            feederPow = 0;

            robot.rfeedMotor.setPower(-feederPow);
            robot.lfeedMotor.setPower(feederPow);
            /*

            robot.gyrodrive.vertical(-0.7, Convert.tileToYeetGV(.5), robot.getHeading());
            robot.gyrodrive.turn(0.7, -180);

            if (colorSensor.red() + colorSensor.blue() + colorSensor.green() > 525) {
                feederPow = -.5;
                robot.rfeedMotor.setPower(-feederPow);
                robot.lfeedMotor.setPower(feederPow);
                wait(2000);
                feederPow = .5;
                robot.gyrodrive.turn(0.7, -90);
                robot.gyrodrive.vertical(0.7, Convert.tileToYeetGV(1), robot.getHeading());
                stoneCount += 1;

            }else{
                skystone = true;
                feederPow = 0;
            }

            robot.rfeedMotor.setPower(-feederPow);
            robot.lfeedMotor.setPower(feederPow);

            telemetry.update();

             */
            break;



        }




    }
}