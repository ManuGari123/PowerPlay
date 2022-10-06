package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Autonomous Without RobotClass", group = "Competition")
public class Autonomous_NoJOEY extends LinearOpMode {

    DcMotor          RFMotor;
    DcMotor          LFMotor;
    DcMotor          RBMotor;
    DcMotor          LBMotor;
    DcMotor          ArmHeight;
    DcMotor          Turret;
//ADD INTAKE AND OUTTAKE CODE HERE
    DistanceSensor   Bottom;


    final double  TicksCount            = 537.7;
    final double  GearRatio             = 1.0;     // No Gearing.
    final double  WheelDiameterIn       = 3.77953;     // For circumference
    double        TickstoInchesWheel    = (TicksCount * GearRatio) / (WheelDiameterIn * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {
//INITIALIZATION + SET MODE:
        LFMotor = hardwareMap.dcMotor.get("LFMotor");
        RFMotor = hardwareMap.dcMotor.get("RFMotor");
        LBMotor = hardwareMap.dcMotor.get("LBMotor");
        RBMotor = hardwareMap.dcMotor.get("RBMotor");
        ArmHeight = hardwareMap.dcMotor.get("ArmHeight");
        Turret = hardwareMap.dcMotor.get("Turret");
        Bottom = hardwareMap.get(DistanceSensor.class, "Bottom");

        LBMotor.setDirection(DcMotor.Direction.REVERSE);
        LFMotor.setDirection(DcMotor.Direction.REVERSE);

        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmHeight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmHeight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//Moving Code Here:
        waitForStart();

        //DISTANCE SENSOR RESET ENCODERS:
        if (opModeIsActive()) {
            if (Bottom.getDistance(DistanceUnit.MM) < 10) {
                ArmHeight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }

        //IF CONE IS PRELOADED:
        int Pos1 = Turret.getCurrentPosition();
        Pos1 += (int) TicksCount/2; //
        Turret.setTargetPosition(Pos1);

        //MOVING + TURRET SAME TIME
        if (Turret.getCurrentPosition() < Pos1 && LFMotor.isBusy()) {
            Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Turret.setPower(0.5);
            FORWARD_ENCODERS_AUTONOMOUS(0.5, 10);
        }

        Turret.setPower(0);


        //Add Outtake Code Here

        int Pos2 = 0; //RESET TO THE MINIMUM POSITION
        Turret.setTargetPosition(Pos1);

        if (Turret.getCurrentPosition() > Pos2) {
            Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Turret.setPower(0.5);
            BACKWARDS_ENCODERS_AUTONOMOUS(0.5, 10);
        }

        //Add Intake here:

        //FOLLOWING CODE IS ONLY TO GET RID OF WARNINGS: DELETE FOLLOWING CODE WHEN TESTING:
        LEFT_ENCODERS_AUTONOMOUS(5,5);
        RIGHT_ENCODERS_AUTONOMOUS(5,5);

    }

    public void FORWARD_ENCODERS_AUTONOMOUS(double speed, double inches) {

        int ForwardDriveLFC = LFMotor.getCurrentPosition() + (int) (inches * TickstoInchesWheel);
        int ForwardDriveRFC = RFMotor.getCurrentPosition() + (int) (inches * TickstoInchesWheel);
        int ForwardDriveLBC = LBMotor.getCurrentPosition() + (int) (inches * TickstoInchesWheel);
        int ForwardDriveRBC = RBMotor.getCurrentPosition() + (int) (inches * TickstoInchesWheel);
        RFMotor.setTargetPosition(ForwardDriveRFC);
        RBMotor.setTargetPosition(ForwardDriveRBC);
        LFMotor.setTargetPosition(ForwardDriveLFC);
        LBMotor.setTargetPosition(ForwardDriveLBC);

        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RFMotor.setPower(Math.abs(speed));
        RBMotor.setPower(Math.abs(speed));
        LFMotor.setPower(Math.abs(speed));
        LBMotor.setPower(Math.abs(speed));

        LBMotor.setPower(0);
        RFMotor.setPower(0);
        RBMotor.setPower(0);
        LFMotor.setPower(0);
    }

    public void BACKWARDS_ENCODERS_AUTONOMOUS(double speed, double inches) {

        int ForwardDriveLFC = LFMotor.getCurrentPosition() + (int) (inches * TickstoInchesWheel);
        int ForwardDriveRFC = RFMotor.getCurrentPosition() + (int) (inches * TickstoInchesWheel);
        int ForwardDriveLBC = LBMotor.getCurrentPosition() + (int) (inches * TickstoInchesWheel);
        int ForwardDriveRBC = RBMotor.getCurrentPosition() + (int) (inches * TickstoInchesWheel);
        RFMotor.setTargetPosition(-ForwardDriveRFC);
        RBMotor.setTargetPosition(-ForwardDriveRBC);
        LFMotor.setTargetPosition(-ForwardDriveLFC);
        LBMotor.setTargetPosition(-ForwardDriveLBC);

        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RFMotor.setPower(Math.abs(speed));
        RBMotor.setPower(Math.abs(speed));
        LFMotor.setPower(Math.abs(speed));
        LBMotor.setPower(Math.abs(speed));

        LBMotor.setPower(0);
        RFMotor.setPower(0);
        RBMotor.setPower(0);
        LFMotor.setPower(0);
    }

    public void LEFT_ENCODERS_AUTONOMOUS(double speed, double inches) {

        int ForwardDriveLFC = LFMotor.getCurrentPosition() + (int) (inches * TickstoInchesWheel);
        int ForwardDriveRFC = RFMotor.getCurrentPosition() + (int) (inches * TickstoInchesWheel);
        int ForwardDriveLBC = LBMotor.getCurrentPosition() + (int) (inches * TickstoInchesWheel);
        int ForwardDriveRBC = RBMotor.getCurrentPosition() + (int) (inches * TickstoInchesWheel);
        RFMotor.setTargetPosition(ForwardDriveRFC);
        RBMotor.setTargetPosition(-ForwardDriveRBC);
        LFMotor.setTargetPosition(-ForwardDriveLFC);
        LBMotor.setTargetPosition(ForwardDriveLBC);

        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RFMotor.setPower(Math.abs(speed));
        RBMotor.setPower(Math.abs(speed));
        LFMotor.setPower(Math.abs(speed));
        LBMotor.setPower(Math.abs(speed));

        LBMotor.setPower(0);
        RFMotor.setPower(0);
        RBMotor.setPower(0);
        LFMotor.setPower(0);
    }

    public void RIGHT_ENCODERS_AUTONOMOUS(double speed, double inches) {

        int ForwardDriveLFC = LFMotor.getCurrentPosition() + (int) (inches * TickstoInchesWheel);
        int ForwardDriveRFC = RFMotor.getCurrentPosition() + (int) (inches * TickstoInchesWheel);
        int ForwardDriveLBC = LBMotor.getCurrentPosition() + (int) (inches * TickstoInchesWheel);
        int ForwardDriveRBC = RBMotor.getCurrentPosition() + (int) (inches * TickstoInchesWheel);
        RFMotor.setTargetPosition(-ForwardDriveRFC);
        RBMotor.setTargetPosition(ForwardDriveRBC);
        LFMotor.setTargetPosition(ForwardDriveLFC);
        LBMotor.setTargetPosition(-ForwardDriveLBC);

        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RFMotor.setPower(Math.abs(speed));
        RBMotor.setPower(Math.abs(speed));
        LFMotor.setPower(Math.abs(speed));
        LBMotor.setPower(Math.abs(speed));

        LBMotor.setPower(0);
        RFMotor.setPower(0);
        RBMotor.setPower(0);
        LFMotor.setPower(0);
    }
}