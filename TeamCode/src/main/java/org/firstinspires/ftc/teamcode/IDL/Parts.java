package org.firstinspires.ftc.teamcode.IDL;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Parts {

    ////////////////////slide orizontal////////////////////////
    public static class HSlide {
        private Servo hSlideServo;

        public HSlide(HardwareMap hardwareMap) {
            hSlideServo = hardwareMap.get(Servo.class, "hSlideServoName");
        }

        public class HSlideOut implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hSlideServo.setPosition(1.0);
                  return false;
            }
        }
        public Action hSlideOut() {
            return new HSlideOut();
        }

        public class HSlideIn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                hSlideServo.setPosition(0.0);
                return false;
            }
        }
        public Action hSlideIn() {
            return new HSlideIn();
        }
    }
////////////////////brat intake////////////////////////
    public static class IntakeArm {
        private Servo intakeArmServo;

        public IntakeArm(HardwareMap hardwareMap) {
            intakeArmServo = hardwareMap.get(Servo.class, "intakeArmServoName");
        }

        public class IntakeArmUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeArmServo.setPosition(1.0);
                return false;
            }
        }
        public Action intakeArmUp() {
            return new IntakeArmUp();
        }

        public class IntakeArmDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeArmServo.setPosition(0.0);
                return false;
            }
        }
        public Action intakeArmDown() {
            return new IntakeArmDown();
        }
    }

    ////////////////////motor intake////////////////////////
    public static class IntakeMotor {
        private CRServo intakeMotorServo;

        public IntakeMotor(HardwareMap hardwareMap) {
            intakeMotorServo = hardwareMap.get(CRServo.class, "intakeMotorServoName");
        }

        public class IntakeMotorStart implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeMotorServo.setPower(1.0);
                return false;
            }
        }
        public Action intakeMotorStart() {
            return new IntakeMotorStart();
        }

        public class IntakeMotorStop implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeMotorServo.setPower(0.0);
                return false;
            }
        }
        public Action intakeMotorStop() {
            return new IntakeMotorStop();
        }
    }

    ////////////////////Lift////////////////////////
    public static class Lift {
        private Servo liftServo;
        private DcMotorEx liftMotor;

        PIDFController pidfController = new PIDFController(kp, ki, kd, kf);

        public static double kp = 0.005, kd = 0.0001, ki = 0, kf = 0.05, target =0;

        public Lift(HardwareMap hardwareMap) {
            liftServo = hardwareMap.get(Servo.class, "liftServoName");
            liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotorName");
        }
        public class LiftDeploy implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                liftServo.setPosition(1.0);
                return false;
            }
        }
        public Action liftDeploy() {
            return new LiftDeploy();
        }
        public class LiftLoad implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                liftServo.setPosition(0.0);
                return false;
            }
        }
        public Action liftLoad() {
            return new LiftLoad();
        }

        public class LiftUpdatePIDF implements Action {
            private boolean initialized = true;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                pidfController.setPIDF(kp, ki, kd, kf);

                int liftPos = (liftMotor.getCurrentPosition());
                liftMotor.setPower(pidfController.calculate(liftPos, target));

                if (!initialized) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        public Action liftUpdatePIDF() {
            return new LiftUpdatePIDF();
        }

        public class LiftUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                target = 2000;
                return false;
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                target = 10;
                return false;
            }
        }
        public Action liftDown() {
            return new LiftDown();
        }

    }
}
