package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Joey {

    public static DcMotor LFMotor;
    public static DcMotor LBMotor;
    public static DcMotor RFMotor;
    public static DcMotor RBMotor;
    public static DcMotor Catapult;
    public static DcMotor ArmHeight;
    public static DcMotor Turret;


    static final double TicksCount = 537.7;    // eg: TETRIX Motor Encoder
    static final double GearRatio = 1.0;     // No External Gearing.
    static final double WheelDiameterIn = 3.77953;     // For figuring circumference
    static final double TickstoInchesWheel = (TicksCount * GearRatio) / (WheelDiameterIn * 3.1415);

    public double MAX_POS = 1.0;
    public double MIN_POS = 0.0;
    public double INCREMENT = 0.01;
    public double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    public boolean Up = gamepad1.dpad_up;
    public boolean Down = gamepad1.dpad_down;
    public boolean Left = gamepad1.dpad_left;
    public boolean Right = gamepad1.dpad_right;


    public void TELEOP_TRIGCONSTANTS() {
        // Power Controls
        double powerMultiply;
        powerMultiply = 1 - gamepad1.right_trigger;

        // Movement Base Gamepad1
        double lateral = gamepad1.left_stick_x;
        double longitudinal = -gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;
        double wheelPower = Math.hypot(lateral, longitudinal);
        double stickAngleRadians = Math.atan2(longitudinal, lateral);
        stickAngleRadians = stickAngleRadians - Math.PI / 4;
        double sinAngleRadians = Math.sin(stickAngleRadians);
        double cosAngleRadians = Math.cos(stickAngleRadians);
        double factor = 1 / Math.max(Math.abs(sinAngleRadians), Math.abs(cosAngleRadians));
        double LFPower = (wheelPower * cosAngleRadians * factor + turn) * powerMultiply;
        Joey.LFMotor.setPower(LFPower);
        double RFPower = (wheelPower * sinAngleRadians * factor - turn) * powerMultiply;
        Joey.RFMotor.setPower(RFPower);
        double LBPower = (wheelPower * sinAngleRadians * factor + turn) * powerMultiply;
        Joey.LBMotor.setPower(LBPower);
        double RBPower = (wheelPower * cosAngleRadians * factor - turn) * powerMultiply;
        Joey.RBMotor.setPower(RBPower);
        boolean Up = gamepad2.dpad_up;
        boolean Down = gamepad2.dpad_down;
        boolean Left = gamepad2.dpad_left;
        boolean Right = gamepad2.dpad_right;

    }

    public void FORWARD_ENCODERS_AUTONOMOUS(double speed, double inches) {

        int ForwardDriveLFC = Joey.LFMotor.getCurrentPosition() + (int) (inches * Joey.TickstoInchesWheel);
        int ForwardDriveRFC = Joey.RFMotor.getCurrentPosition() + (int) (inches * Joey.TickstoInchesWheel);
        int ForwardDriveLBC = Joey.LBMotor.getCurrentPosition() + (int) (inches * Joey.TickstoInchesWheel);
        int ForwardDriveRBC = Joey.RBMotor.getCurrentPosition() + (int) (inches * Joey.TickstoInchesWheel);
        Joey.RFMotor.setTargetPosition(ForwardDriveRFC);
        Joey.RBMotor.setTargetPosition(ForwardDriveRBC);
        Joey.LFMotor.setTargetPosition(ForwardDriveLFC);
        Joey.LBMotor.setTargetPosition(ForwardDriveLBC);

        Joey.RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Joey.RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Joey.LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Joey.LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Joey.RFMotor.setPower(Math.abs(speed));
        Joey.RBMotor.setPower(Math.abs(speed));
        Joey.LFMotor.setPower(Math.abs(speed));
        Joey.LBMotor.setPower(Math.abs(speed));

        LBMotor.setPower(0);
        RFMotor.setPower(0);
        RBMotor.setPower(0);
        LFMotor.setPower(0);
    }

    public void LEFT_ENCODERS_AUTONOMOUS(double speed, double inches) {

        int ForwardDriveLFC = Joey.LFMotor.getCurrentPosition() + (int) (inches * Joey.TickstoInchesWheel);
        int ForwardDriveRFC = Joey.RFMotor.getCurrentPosition() + (int) (inches * Joey.TickstoInchesWheel);
        int ForwardDriveLBC = Joey.LBMotor.getCurrentPosition() + (int) (inches * Joey.TickstoInchesWheel);
        int ForwardDriveRBC = Joey.RBMotor.getCurrentPosition() + (int) (inches * Joey.TickstoInchesWheel);
        Joey.RFMotor.setTargetPosition(ForwardDriveRFC);
        Joey.RBMotor.setTargetPosition(-ForwardDriveRBC);
        Joey.LFMotor.setTargetPosition(-ForwardDriveLFC);
        Joey.LBMotor.setTargetPosition(ForwardDriveLBC);

        Joey.RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Joey.RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Joey.LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Joey.LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Joey.RFMotor.setPower(Math.abs(speed));
        Joey.RBMotor.setPower(Math.abs(speed));
        Joey.LFMotor.setPower(Math.abs(speed));
        Joey.LBMotor.setPower(Math.abs(speed));

        LBMotor.setPower(0);
        RFMotor.setPower(0);
        RBMotor.setPower(0);
        LFMotor.setPower(0);
    }

    public void RIGHT_ENCODERS_AUTONOMOUS(double speed, double inches) {

        int ForwardDriveLFC = Joey.LFMotor.getCurrentPosition() + (int) (inches * Joey.TickstoInchesWheel);
        int ForwardDriveRFC = Joey.RFMotor.getCurrentPosition() + (int) (inches * Joey.TickstoInchesWheel);
        int ForwardDriveLBC = Joey.LBMotor.getCurrentPosition() + (int) (inches * Joey.TickstoInchesWheel);
        int ForwardDriveRBC = Joey.RBMotor.getCurrentPosition() + (int) (inches * Joey.TickstoInchesWheel);
        Joey.RFMotor.setTargetPosition(-ForwardDriveRFC);
        Joey.RBMotor.setTargetPosition(ForwardDriveRBC);
        Joey.LFMotor.setTargetPosition(ForwardDriveLFC);
        Joey.LBMotor.setTargetPosition(-ForwardDriveLBC);

        Joey.RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Joey.RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Joey.LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Joey.LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Joey.RFMotor.setPower(Math.abs(speed));
        Joey.RBMotor.setPower(Math.abs(speed));
        Joey.LFMotor.setPower(Math.abs(speed));
        Joey.LBMotor.setPower(Math.abs(speed));

        LBMotor.setPower(0);
        RFMotor.setPower(0);
        RBMotor.setPower(0);
        LFMotor.setPower(0);
    }

    public void BACKWARDS_ENCODERS_AUTONOMOUS(double speed, double inches) {

        int ForwardDriveLFC = Joey.LFMotor.getCurrentPosition() + (int) (inches * Joey.TickstoInchesWheel);
        int ForwardDriveRFC = Joey.RFMotor.getCurrentPosition() + (int) (inches * Joey.TickstoInchesWheel);
        int ForwardDriveLBC = Joey.LBMotor.getCurrentPosition() + (int) (inches * Joey.TickstoInchesWheel);
        int ForwardDriveRBC = Joey.RBMotor.getCurrentPosition() + (int) (inches * Joey.TickstoInchesWheel);
        Joey.RFMotor.setTargetPosition(-ForwardDriveRFC);
        Joey.RBMotor.setTargetPosition(-ForwardDriveRBC);
        Joey.LFMotor.setTargetPosition(-ForwardDriveLFC);
        Joey.LBMotor.setTargetPosition(-ForwardDriveLBC);

        Joey.RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Joey.RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Joey.LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Joey.LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Joey.RFMotor.setPower(Math.abs(speed));
        Joey.RBMotor.setPower(Math.abs(speed));
        Joey.LFMotor.setPower(Math.abs(speed));
        Joey.LBMotor.setPower(Math.abs(speed));

        LBMotor.setPower(0);
        RFMotor.setPower(0);
        RBMotor.setPower(0);
        LFMotor.setPower(0);
    }

    public void HARDWARE_INITIALIZE_BASE() {
        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        LFMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void ENCODERS_STARTUP() {
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Starting at", "%7d :%7d",
                LFMotor.getCurrentPosition(),
                LBMotor.getCurrentPosition(),
                RFMotor.getCurrentPosition(),
                RBMotor.getCurrentPosition());

        telemetry.update();
    }

    public void HOMECOMING_HARDWARE_INITIALIZE_CATAPULT() {

        Catapult = hardwareMap.dcMotor.get("Catapult");
    }

    public void HARDWARE_INITIALIZE_TURRET_ARMHEIGHT() {
        Turret = hardwareMap.dcMotor.get("Turret");
        ArmHeight = hardwareMap.dcMotor.get("ArmHeight");
    }
}