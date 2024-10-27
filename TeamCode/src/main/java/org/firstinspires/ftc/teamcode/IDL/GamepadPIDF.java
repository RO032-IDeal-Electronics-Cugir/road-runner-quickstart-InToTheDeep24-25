
package org.firstinspires.ftc.teamcode.IDL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

//push test elev
//push test je
@Config
@TeleOp(name = "TunePID")
public class GamepadPIDF extends OpMode {
    private PIDFController controllerPIDF;

    public static double p=0, i=0, d=0, f=0;
    double power;
    int liftPos;
    private DcMotorEx liftMotor;
    public static int liftTarget=500;
    boolean usePIDF = false;
    Gamepad lastGamepad1 = new Gamepad();
    //Gamepad lastGamepad2 = new Gamepad();

    @Override
    public void init(){
        controllerPIDF = new PIDFController(p, i, d, f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotorName");
    }

    public void loop(){
        //controllerPIDF.setPIDF(p,i,d,f);
        // This is a rising-edge detector that runs if and only if "a" was pressed this loop.
        if (gamepad1.a && !lastGamepad1.a) {
            usePIDF = true;
        }

        if (gamepad1.left_trigger > 0) {
            power = gamepad1.left_trigger;
            liftMotor.setPower(power);

            // If we get any sort of manual input, turn PIDF off.
            usePIDF = false;
        } else if (gamepad1.right_trigger > 0) {
            power = gamepad1.right_trigger;
            liftMotor.setPower(power);

            // If we get any sort of manual input, turn PIDF off.
            usePIDF = false;
        } else if (usePIDF) {
            // Sets the slide motor power according to the PIDF output.
            int liftPos = liftMotor.getCurrentPosition();
            power = controllerPIDF.calculate(liftPos, liftTarget);
            liftMotor.setPower(power);
        }
        //liftMotor.setPower(power);
        telemetry.addData("pos ", liftPos);
        telemetry.addData("liftTarget ", liftTarget);
        telemetry.addData("power ", power);
        telemetry.update();
    }
}