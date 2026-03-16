package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;

public class Hanger extends SubsystemBase {
    public enum Position {
        HOMED(0),
        EXTEND_HOPPER(2),
        HANGING(6),
        HUNG(0.2);

        private final double inches;

        private Position(double inches) {
            this.inches = inches;
        }

        public Angle motorAngle() {
            final Measure<AngleUnit> angleMeasure = Inches.of(inches).divideRatio(kHangerExtensionPerMotorAngle);
            return Rotations.of(angleMeasure.in(Rotations)); // Promote from AngleUnit to Angle
        }
    }

    private static final Per<DistanceUnit, AngleUnit> kHangerExtensionPerMotorAngle = Inches.of(6).div(Rotations.of(142));
    private static final Distance kExtensionTolerance = Inches.of(1);
    private static final Distance kExtensionTolerance2 = Inches.of(1);

    private final TalonFX motor;
    private final TalonFX motor2;
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private boolean isHomed = false;

    public Hanger() {
        motor = new TalonFX(Ports.kHanger, Ports.kRoboRioCANBus);
        motor2 = new TalonFX(Ports.kHanger2, Ports.kRoboRioCANBus);
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(20))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(KrakenX60.kFreeSpeed)
                    .withMotionMagicAcceleration(KrakenX60.kFreeSpeed.per(Second))
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(10)
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );

        final TalonFXConfiguration config2 = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(20))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(KrakenX60.kFreeSpeed)
                    .withMotionMagicAcceleration(KrakenX60.kFreeSpeed.per(Second))
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(10)
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / KrakenX60.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );

        motor.getConfigurator().apply(config);
        motor2.getConfigurator().apply(config2);
        SmartDashboard.putData(this);
    }

    public void set(Position position) {
        motor.setControl(
            motionMagicRequest
                .withPosition(position.motorAngle())
        );
        motor2.setControl(
            motionMagicRequest
                .withPosition(position.motorAngle())
        );
    }

    public void setPercentOutput(double percentOutput) {
        motor.setControl(
            voltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
        motor2.setControl(
            voltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }

    //create another position command for motor 2
    public Command positionCommand(Position position) {
        return runOnce(() -> set(position))
            .andThen(Commands.waitUntil(this::isExtensionWithinTolerance));
            
    }

    public Command homingCommand() {
        return Commands.sequence(
            runOnce(() -> setPercentOutput(-0.05)),
            Commands.waitUntil(() -> motor.getSupplyCurrent().getValue().in(Amps) > 0.4),
            runOnce(() -> {
                motor.setPosition(Position.HOMED.motorAngle());
                isHomed = true;
                set(Position.EXTEND_HOPPER);
                 
            })
        )
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
    //half solution idk 
    public Command homingCommand2() {
        return Commands.sequence(
            runOnce(() -> setPercentOutput(-0.05)),
            Commands.waitUntil(() -> motor2.getSupplyCurrent().getValue().in(Amps) > 0.4),
            runOnce(() -> {
                motor.setPosition(Position.HOMED.motorAngle());
                isHomed = true;
                set(Position.EXTEND_HOPPER);
                 
            })
        )
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }


    public boolean isHomed() {
        return isHomed;
    }
    // take a look at this for auto/climber when it is working
    private boolean isExtensionWithinTolerance() {
        final Distance currentExtension = motorAngleToExtension(motor.getPosition().getValue());
        final Distance targetExtension = motorAngleToExtension(motionMagicRequest.getPositionMeasure());
        return currentExtension.isNear(targetExtension, kExtensionTolerance);
         
    }

    // test solution
    private boolean isExtensionWithinTolerance2(){
        final Distance currentExtension2 = motorAngleToExtension(motor2.getPosition().getValue());
        final Distance targetExtension2 = motorAngleToExtension(motionMagicRequest.getPositionMeasure());
        return currentExtension2.isNear(targetExtension2, kExtensionTolerance2);
    }

//make another motorangleextension for motor 2
    private Distance motorAngleToExtension(Angle motorAngle) {
        final Measure<DistanceUnit> extensionMeasure = motorAngle.timesRatio(kHangerExtensionPerMotorAngle);
        return Inches.of(extensionMeasure.in(Inches)); // Promote from DistanceUnit to Distance
    }

    // test solution 
    private Distance motorAngletoExtension2(Angle motorAngle2) {
        final Measure<DistanceUnit> extensionMeasure2 = motorAngle2.timesRatio(kHangerExtensionPerMotorAngle);
        return Inches.of(extensionMeasure2.in(Inches)); 
    }
    // write motor2 lines for the sendables as well
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Extension (inches)", () -> motorAngleToExtension(motor.getPosition().getValue()).in(Inches), null);
        builder.addDoubleProperty("Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
    }
}
