package first.robot.v3_Horse.subsystems.intake;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.subsystems.extension.ExtensionConstants;
import edu.wpi.team190.gompeilib.subsystems.extension.ExtensionConstants.ExtensionParameters;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerConstants;
import java.util.Map;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.Units;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Voltage;

public class V3_HorseIntakeConstants {
  public static final ExtensionConstants LEFT_EXTENSION_CONSTANTS;
  public static final ExtensionConstants RIGHT_EXTENSION_CONSTANTS;

  public static final GenericRollerConstants LEFT_INTAKE_ROLLER_CONSTANTS;
  public static final GenericRollerConstants RIGHT_INTAKE_ROLLER_CONSTANTS;
  public static final GenericRollerConstants KICKER_ROLLER_CONSTANTS;

  public static final double MAX_EXTENSION;
  public static final double MIN_EXTENSION;

  public static final double EXTENSION_STOW_POSITION;
  public static final double EXTENSION_INTAKE_POSITION;

  public static final double AGITATE_AUTO_STOW_THRESHOLD;

  public static final double TRIGGER_CURVE;

  public static final Voltage INTAKE_IN_ROLLER_VOLTAGE;
  public static final Voltage KICKER_IN_VOLTAGE;

  public static final Voltage INTAKE_OUT_ROLLER_VOLTAGE;
  public static final Voltage KICKER_OUT_VOLTAGE;

  public static final Voltage IDLE_ROLLER_VOLTAGE;

  public static final Voltage EXTENSION_SLOW_MOVE_VOLTAGE;

  public static final Voltage EXTENSION_VOLTAGE_OFFSET_STEP;

  // SETPOINT TIME

  public static final Map<ExtensionState, Setpoint<DistanceUnit>> LEFT_EXTENSION_STATES;
  public static final Map<ExtensionState, Setpoint<DistanceUnit>> RIGHT_EXTENSION_STATES;

  public static final Map<RollerState, Setpoint<VoltageUnit>> INTAKE_ROLLER_STATES;
  public static final Map<RollerState, Setpoint<VoltageUnit>> KICKER_ROLLER_STATES;

  static {
    MAX_EXTENSION = 0.3;
    MIN_EXTENSION = 0.0;

    EXTENSION_STOW_POSITION = 0.0;
    EXTENSION_INTAKE_POSITION = 0.3;

    AGITATE_AUTO_STOW_THRESHOLD = 0.01;

    TRIGGER_CURVE = 0.0;

    INTAKE_IN_ROLLER_VOLTAGE = Volts.of(12);
    INTAKE_OUT_ROLLER_VOLTAGE = Volts.of(-12);

    KICKER_IN_VOLTAGE = Volts.of(12);
    KICKER_OUT_VOLTAGE = Volts.of(-12);

    IDLE_ROLLER_VOLTAGE = Volts.of(5);

    EXTENSION_SLOW_MOVE_VOLTAGE = Volts.of(1);

    EXTENSION_VOLTAGE_OFFSET_STEP = Volts.of(1);

    LEFT_EXTENSION_CONSTANTS =
        ExtensionConstants.builder()
            .withLeaderCANID(21)
            .withExtensionGearRatio((52.0 / 30) * (52.0 / 16))
            .withLeaderInvertedValue(InvertedValue.CounterClockwise_Positive)
            .withDrumRadius(0.0127)
            .withExtensionSupplyCurrentLimit(40.0)
            .withExtensionStatorCurrentLimit(40.0)
            .withSlot0Gains(
                Gains.builder()
                    .withKP(10)
                    .withKD(0.0)
                    .withKS(0.42002)
                    .withKV(6.3697)
                    .withKA(0.0)
                    .build())
            .withSlot1Gains(
                Gains.builder().withKP(0.1).withKD(0.0).withKS(0.0).withKV(0.0).withKA(0.0).build())
            .withSlot2Gains(Gains.builder().build())
            .withVerticalGravity(false)
            .withVoltageOffsetStep(EXTENSION_VOLTAGE_OFFSET_STEP)
            .withHeightOffsetStep(Meters.of(0.01))
            .withCanBus(new CANBus(CANPort.CAN_S0))
            .withExtensionParameters(
                ExtensionParameters.builder()
                    .withCARRIAGE_MASS_KG(10.0)
                    .withEXTENSION_MOTOR_CONFIG(DCMotor.getKrakenX44Foc(1))
                    .withMAX_LENGTH(Meters.of(MAX_EXTENSION))
                    .withMIN_LENGTH(Meters.of(MIN_EXTENSION))
                    .withNUM_MOTORS(1)
                    .build())
            .withConstraints(
                LinearConstraints.builder()
                    .withPrefix("Intake/Left Extension/Constraints")
                    .withGoalTolerance(Meters.of(0.01))
                    .withMaxAcceleration(MetersPerSecondPerSecond.of(20.0))
                    .withMaxVelocity(MetersPerSecond.of(20))
                    .build())
            .build();

    RIGHT_EXTENSION_CONSTANTS =
        ExtensionConstants.builder()
            .withLeaderCANID(22)
            .withExtensionGearRatio((52.0 / 30) * (52.0 / 16))
            .withLeaderInvertedValue(InvertedValue.Clockwise_Positive)
            .withDrumRadius(0.0127)
            .withExtensionSupplyCurrentLimit(40.0)
            .withExtensionStatorCurrentLimit(40.0)
            .withSlot0Gains(
                Gains.builder()
                    .withKP(10)
                    .withKD(0.0)
                    .withKS(0.42002)
                    .withKV(6.3697)
                    .withKA(0.0)
                    .build())
            .withSlot1Gains(
                Gains.builder().withKP(0.1).withKD(0.0).withKS(0.0).withKV(0.0).withKA(0.0).build())
            .withSlot2Gains(Gains.builder().build())
            .withVerticalGravity(false)
            .withVoltageOffsetStep(EXTENSION_VOLTAGE_OFFSET_STEP)
            .withHeightOffsetStep(Meters.of(0.01))
            .withCanBus(new CANBus(CANPort.CAN_S0))
            .withExtensionParameters(
                ExtensionParameters.builder()
                    .withCARRIAGE_MASS_KG(10.0)
                    .withEXTENSION_MOTOR_CONFIG(DCMotor.getKrakenX44Foc(1))
                    .withMAX_LENGTH(Meters.of(MAX_EXTENSION))
                    .withMIN_LENGTH(Meters.of(MIN_EXTENSION))
                    .withNUM_MOTORS(1)
                    .build())
            .withConstraints(
                LinearConstraints.builder()
                    .withPrefix("Intake/Right Extension/Constraints")
                    .withGoalTolerance(Meters.of(0.01))
                    .withMaxAcceleration(MetersPerSecondPerSecond.of(20.0))
                    .withMaxVelocity(MetersPerSecond.of(20))
                    .build())
            .build();

    LEFT_INTAKE_ROLLER_CONSTANTS =
        GenericRollerConstants.builder()
            .withLeaderCANID(23)
            .withCurrentLimits(
                CurrentLimits.builder()
                    .withStatorCurrentLimit(Amps.of(40.0))
                    .withSupplyCurrentLimit(Amps.of(40.0))
                    .build())
            .withNeutralMode(NeutralModeValue.Coast)
            .withRollerGearbox(DCMotor.getKrakenX60Foc(1))
            .withRollerMotorGearRatio(36.0 / 12)
            .withLeaderInvertedValue(InvertedValue.CounterClockwise_Positive)
            .withMomentOfInertia(Units.KilogramSquareMeters.of(0.0004))
            .withVoltageOffsetStep(Volts.of(1))
            .withCanBus(new CANBus(CANPort.CAN_S2))
            .withEnableFOC(false)
            .build();

    RIGHT_INTAKE_ROLLER_CONSTANTS =
        GenericRollerConstants.builder()
            .withLeaderCANID(24)
            .withCurrentLimits(
                CurrentLimits.builder()
                    .withStatorCurrentLimit(Amps.of(40.0))
                    .withSupplyCurrentLimit(Amps.of(40.0))
                    .build())
            .withNeutralMode(NeutralModeValue.Coast)
            .withRollerGearbox(DCMotor.getKrakenX60Foc(1))
            .withRollerMotorGearRatio(36.0 / 12)
            .withLeaderInvertedValue(InvertedValue.Clockwise_Positive)
            .withMomentOfInertia(Units.KilogramSquareMeters.of(0.0004))
            .withVoltageOffsetStep(Volts.of(1))
            .withCanBus(new CANBus(CANPort.CAN_S1))
            .withEnableFOC(false)
            .build();

    KICKER_ROLLER_CONSTANTS =
        GenericRollerConstants.builder()
            .withLeaderCANID(25)
            .withCurrentLimits(
                CurrentLimits.builder()
                    .withStatorCurrentLimit(Amps.of(40.0))
                    .withSupplyCurrentLimit(Amps.of(40.0))
                    .build())
            .withNeutralMode(NeutralModeValue.Coast)
            .withRollerGearbox(DCMotor.getKrakenX60Foc(1))
            .withRollerMotorGearRatio((58.0 / 26.0) * (24.0 / 18.0))
            .withLeaderInvertedValue(InvertedValue.CounterClockwise_Positive)
            .withMomentOfInertia(Units.KilogramSquareMeters.of(0.0004))
            .withVoltageOffsetStep(Volts.of(1))
            .withCanBus(new CANBus(CANPort.CAN_S0))
            .withEnableFOC(false)
            .build();

    LEFT_EXTENSION_STATES =
        Map.of(
            ExtensionState.STOW,
            new Setpoint<>(
                Meters.of(EXTENSION_STOW_POSITION),
                Meters.of(0.01),
                Meters.of(MIN_EXTENSION),
                Meters.of(MAX_EXTENSION)),
            ExtensionState.INTAKE,
            new Setpoint<>(
                Meters.of(EXTENSION_INTAKE_POSITION),
                Meters.of(0.01),
                Meters.of(MIN_EXTENSION),
                Meters.of(MAX_EXTENSION)));

    RIGHT_EXTENSION_STATES =
        Map.of(
            ExtensionState.STOW,
            new Setpoint<>(
                Meters.of(EXTENSION_STOW_POSITION),
                Meters.of(0.01),
                Meters.of(MIN_EXTENSION),
                Meters.of(MAX_EXTENSION)),
            ExtensionState.INTAKE,
            new Setpoint<>(
                Meters.of(EXTENSION_INTAKE_POSITION),
                Meters.of(0.01),
                Meters.of(MIN_EXTENSION),
                Meters.of(MAX_EXTENSION)));

    INTAKE_ROLLER_STATES =
        Map.of(
            RollerState.INTAKE,
            new Setpoint<>(
                INTAKE_IN_ROLLER_VOLTAGE,
                LEFT_INTAKE_ROLLER_CONSTANTS.voltageOffsetStep,
                Volts.of(-12.0),
                Volts.of(12.0)),
            RollerState.EXTAKE,
            new Setpoint<>(
                INTAKE_OUT_ROLLER_VOLTAGE,
                LEFT_INTAKE_ROLLER_CONSTANTS.voltageOffsetStep,
                Volts.of(-12.0),
                Volts.of(12.0)),
            RollerState.IDLE,
            new Setpoint<>(
                IDLE_ROLLER_VOLTAGE,
                LEFT_INTAKE_ROLLER_CONSTANTS.voltageOffsetStep,
                Volts.of(-12.0),
                Volts.of(12.0)),
            RollerState.STOP,
            new Setpoint<>(
                Volts.of(0.0),
                LEFT_INTAKE_ROLLER_CONSTANTS.voltageOffsetStep,
                Volts.of(-12.0),
                Volts.of(12.0)));

    KICKER_ROLLER_STATES =
        Map.of(
            RollerState.INTAKE,
            new Setpoint<>(
                KICKER_IN_VOLTAGE,
                KICKER_ROLLER_CONSTANTS.voltageOffsetStep,
                Volts.of(-12.0),
                Volts.of(12.0)),
            RollerState.EXTAKE,
            new Setpoint<>(
                KICKER_OUT_VOLTAGE,
                KICKER_ROLLER_CONSTANTS.voltageOffsetStep,
                Volts.of(-12.0),
                Volts.of(12.0)),
            RollerState.IDLE,
            new Setpoint<>(
                IDLE_ROLLER_VOLTAGE,
                KICKER_ROLLER_CONSTANTS.voltageOffsetStep,
                Volts.of(-12.0),
                Volts.of(12.0)),
            RollerState.STOP,
            new Setpoint<>(
                Volts.of(0.0),
                KICKER_ROLLER_CONSTANTS.voltageOffsetStep,
                Volts.of(-12.0),
                Volts.of(12.0)));
  }

  public enum ExtensionState {
    STOW,
    INTAKE,
    MANUAL_EXTEND,
    ZERO,
    STOP,
    SLOW_MOVE,
    SQUISH,
    SYS_ID
  }

  public enum RollerState {
    INTAKE,
    IDLE,
    EXTAKE,
    STOP
  }
}
