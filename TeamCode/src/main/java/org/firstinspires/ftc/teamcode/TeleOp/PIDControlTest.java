package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "PID Control Arm", group = "ArmCodeTest")
public class PIDControlTest extends LinearOpMode {

    DcMotorEx ArmHeight;
    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor RBMotor;
    DcMotor LBMotor;
    Servo Turret;
    TouchSensor BottomReset;
    DistanceSensor Pole;

    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double INCREMENT = 0.01;
    double max_pos_turret = 1.0;
    double min_pos_turret = 0;
    double home_turret = (max_pos_turret - min_pos_turret) / 2; //Starts exactly in between
    double turret_increment = INCREMENT;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        ArmHeight = hardwareMap.get(DcMotorEx.class, "ArmHeight");
        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        Turret = hardwareMap.servo.get("Turret");
        BottomReset = hardwareMap.get(TouchSensor.class, "BottomReset");
        Pole = hardwareMap.get(DistanceSensor.class, "Pole");
        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        LFMotor.setDirection(DcMotor.Direction.REVERSE);
        ArmHeight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        while (opModeIsActive()) {
            //Code goes here:

            // Movement Base Gamepad1
            double powerMultiply;
            powerMultiply = 1 - gamepad1.right_trigger;

            // Movement Base Gamepad1
            double lateral = gamepad1.left_stick_x;
            double longitudinal = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double wheelPower = Math.hypot(lateral, longitudinal);
            double stickAngleRadians = Math.atan2(longitudinal, lateral);
            stickAngleRadians = stickAngleRadians - Math.PI / 4;
            double sinAngleRadians = Math.sin(stickAngleRadians);
            double cosAngleRadians = Math.cos(stickAngleRadians);
            double factor = 1 / Math.max(Math.abs(sinAngleRadians), Math.abs(cosAngleRadians));
            double LFPower = (wheelPower * cosAngleRadians * factor + turn) * powerMultiply;
            LFMotor.setPower(LFPower);
            double RFPower = (wheelPower * sinAngleRadians * factor - turn) * powerMultiply;
            RFMotor.setPower(RFPower);
            double LBPower = (wheelPower * sinAngleRadians * factor + turn) * powerMultiply;
            LBMotor.setPower(LBPower);
            double RBPower = (wheelPower * cosAngleRadians * factor - turn) * powerMultiply;
            RBMotor.setPower(RBPower);

            //Turret Left
            while (gamepad2.left_stick_x < 0) {
                home_turret += turret_increment;
                Turret.setPosition(home_turret);
            }

            //Turret Right
            while (gamepad2.left_stick_x > 0) {
                home_turret -= turret_increment;
                Turret.setPosition(home_turret);
            }


            if (gamepad2.a) {
                ArmHeight.setPower(PIDControl(100, ArmHeight.getCurrentPosition()));
                sleep(5000);
                //Figure out exact ticks amount to set for pole height.
                //Figure out and Put in touch sensor controls.
                //If statement for Touch controls, and servo's dropping cone.
                ArmHeight.setPower(PIDControl(-100, ArmHeight.getCurrentPosition()));

            }
            if (gamepad2.b) {
                ArmHeight.setPower(PIDControl(200, ArmHeight.getCurrentPosition()));
                sleep(5000);
                //Figure out exact ticks amount to set for pole height.
                //Figure out and put in touch sensor controls.
                //If statement for Touch controls, and servo's dropping cone.
                ArmHeight.setPower(PIDControl(-100, ArmHeight.getCurrentPosition()));
            }
            if (gamepad2.x) {
                ArmHeight.setPower(PIDControl(300, ArmHeight.getCurrentPosition()));
                sleep(5000);
                //Figure out exact ticks amount to set for pole height.
                //Figure out and Put in touch sensor controls.
                //If statement for Touch controls, and servo's dropping cone.
                ArmHeight.setPower(PIDControl(-100, ArmHeight.getCurrentPosition()));
            }
        }
    }

    public double PIDControl(double ticks, double CurrentPosition) {
        double error = ticks - CurrentPosition;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
}

