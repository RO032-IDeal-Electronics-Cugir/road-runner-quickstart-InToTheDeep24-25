
package org.firstinspires.ftc.teamcode.IDL;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "TunePIDF")
public class TunePIDF extends OpMode {
    private PIDFController controller;
    public static double p=0, i=0, d=0, f=0;
    public static int liftTarget=0;
    private DcMotorEx liftMotor;

    @Override
    public void init(){
        controller = new PIDFController(p,i,d,f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotorName");
    }

    public void loop(){
        controller.setPIDF(p, i, d, f);
        int liftPos = liftMotor.getCurrentPosition();
        double power = controller.calculate(liftPos, liftTarget);
        liftMotor.setPower(power);
        telemetry.addData("pos ", liftPos);
        telemetry.addData("liftTarget ", liftTarget);
        telemetry.addData("power ", power);
        telemetry.update();
    }
}