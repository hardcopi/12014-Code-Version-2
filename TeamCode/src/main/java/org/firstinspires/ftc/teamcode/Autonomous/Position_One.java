package org.firstinspires.ftc.teamcode.Autonomous;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareFireWiresBot;

@Autonomous(name = "Position One", group = "FireBot")
public class Position_One extends LinearOpMode {
    HardwareFireWiresBot robot = new HardwareFireWiresBot();
    long start_time;
    private ElapsedTime runtime = new ElapsedTime();
    static final double SHOOTER_POWER = .25;
    private String particlePref;
    private String beaconPref;
    private String capBallPref;
    private String parkingPref;
    private String alliance;

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 1;
    static final double TURN_SPEED = 0.5;

    static final double FLOOR_REFLECTANCE = 0.2;
    static final double LINE_REFLECTANCE = 0.04;
    static final double THRESHOLD_REFLECTANCE = (LINE_REFLECTANCE + FLOOR_REFLECTANCE) / 2;
    static final double COLOR_THRESHOLD = 1;
    double reflectance = 0.0;

    static int bluecolor;
    static int redcolor;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.color.enableLed(false);
        robot.ods.enableLed(true);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();
        waitForStart();
        getAutonomousPrefs();

        telemetry.addLine("Particles: " + particlePref);
        telemetry.addLine("Beacons: " + beaconPref);
        telemetry.addLine("Cap Ball: " + capBallPref);
        telemetry.addLine("Parking: " + parkingPref);
        telemetry.addLine("Alliance: " + alliance);
        telemetry.update();
        //Initial stuff

        while (opModeIsActive()) {
            switch (alliance) {
                case "Blue Alliance":
                        if (!beaconPref.equalsIgnoreCase("Do not activate any beacons")) {
                            encoderDrive(DRIVE_SPEED, 14, 14, 6);      /* Drive forward */
                            shoot2();
                            encoderDrive(TURN_SPEED, 2, -1, 6);     /* Turn Left */
                            encoderDrive(DRIVE_SPEED, 30, 30, 15);      /* Drive forward */
                            boolean drive = false;

                            /* Try 3 times before moving on */
                            bumpBeacon();
                            sleep(1000);
                            if (robot.color.blue() < COLOR_THRESHOLD) { sleep(4000); bumpBeacon(); } else { updateColors(); }
                            if (robot.color.blue() < COLOR_THRESHOLD) { sleep(4000); bumpBeacon(); } else { updateColors(); }
                            encoderDrive(DRIVE_SPEED, -15, -15, 20);      /* Drive forward */
                        stop();
                    }
                    break;
                case "Red Alliance":
                        stop();
            }
        }
    }

    public void shoot2(){
        robot.leftShooter.setPower(SHOOTER_POWER);
        robot.rightShooter.setPower(SHOOTER_POWER);
        sleep(1000);
        robot.shootServo.setPosition(-1);
        sleep(500);
        robot.shootServo.setPosition(1);
        robot.leftShooter.setPower(0);
        robot.rightShooter.setPower(0);
        sleep(300);
        /* Fire again! */
        robot.shootServo.setPosition(1);
        sleep(1000);
        robot.leftShooter.setPower(SHOOTER_POWER);
        robot.rightShooter.setPower(SHOOTER_POWER);
        sleep(1000);
        robot.shootServo.setPosition(-1);
        sleep(1000);
        robot.leftShooter.setPower(0);
        robot.rightShooter.setPower(0);
    }
    private void getAutonomousPrefs()
    {
        SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        particlePref = preferences.getString("How Many Particles Should We Shoot?", "");
        beaconPref = preferences.getString("Which beacons should we activate?", "");
        capBallPref = preferences.getString("Should we bump the cap ball off the center vortex?", "");
        parkingPref = preferences.getString("Where should we park?", "");
        alliance = preferences.getString("Which alliance are we on?", "");
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches, float timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        leftInches = -leftInches;
        rightInches = -rightInches;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }//

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
    public int getRedColor(){
        return robot.color.red();
    }
    public int getBlueColor(){
        return robot.color.blue();
    }
    public  void updateColors(){
        bluecolor = robot.color.blue();
        redcolor = robot.color.red();
        telemetry.addData("Color Sensor", "Blue: " + robot.color.blue());
        telemetry.addData("Color Sensor", "Red: " + robot.color.red());
        telemetry.update();
    }
    public void hitBeacon(){
        while(opModeIsActive()) {
            sleep(500);
            bluecolor = robot.color.blue();
            telemetry.addData("Red", bluecolor);
            telemetry.addData("Blue", robot.color.blue());
            telemetry.update();
            Robot_Methods.driveForSecondsAtPower(robot, -DRIVE_SPEED, .5);
            sleep(500);
            if(bluecolor > 0){
                telemetry.addData("Found!", robot.color.blue());
                telemetry.update();
                Robot_Methods.driveForSecondsAtPower(robot, DRIVE_SPEED, 1);
                sleep(3000);
                break;
            } else {
                sleep(300);
                bluecolor = robot.color.blue();
                if(robot.color.blue() > 0){
                    telemetry.addData("Found!", robot.color.blue());
                    telemetry.update();
                    Robot_Methods.driveForSecondsAtPower(robot, DRIVE_SPEED, 1);
                    break;
                } else {
                    telemetry.addData("Not Found!", robot.color.blue());
                    telemetry.update();
                }

            }
            updateColors();
            //reverse after hitting
        }
    }

    public void bumpBeacon() {
        boolean ran = false;
        Robot_Methods.driveForSecondsAtPower(robot, -.3, .6); /* Drive Forward 300ms */
        Robot_Methods.driveForSecondsAtPower(robot, .3, .4);  /* Drive Back 300ms */
        telemetry.addData("Bump Beacon", "Blue: " + robot.color.blue());
        telemetry.addData("Bump Beacon", "Red: " + robot.color.red());
        telemetry.update();
    }
}
