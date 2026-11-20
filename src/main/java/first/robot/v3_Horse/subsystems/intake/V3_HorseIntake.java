package first.robot.v3_Horse.subsystems.intake;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Volts;

import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.subsystems.extension.Extension;
import edu.wpi.team190.gompeilib.subsystems.extension.ExtensionIO;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntakeConstants.ExtensionState;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntakeConstants.RollerState;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.measure.Distance;

public class V3_HorseIntake extends SubsystemBase {
  @Getter private ExtensionState extensionState;
  @Getter private RollerState rollerState;

  private final GenericRoller rightIntakeRoller;
  private final GenericRoller leftIntakeRoller;

  public final Extension rightExtension;
  public final Extension leftExtension;

  private final GenericRoller kickerRoller;

  private DoubleSupplier triggerSupplier;

  private final Setpoint<DistanceUnit> leftManualPositionGoal;
  private final Setpoint<DistanceUnit> rightManualPositionGoal;

  @Getter private Distance extensionPosition;

  private boolean manualReachedStow;

  private ExtensionState previousExtensionState;
  private boolean leftCompliant;
  private boolean rightCompliant;

  private GainSlot leftGainSlot;
  private GainSlot rightGainSlot;

  public V3_HorseIntake(
      GenericRollerIO leftIntakeRollerIO,
      GenericRollerIO rightIntakeRollerIO,
      GenericRollerIO kickerRollerIO,
      ExtensionIO rightExtensionIO,
      ExtensionIO leftExtensionIO,
      DoubleSupplier triggerSupplier) {

    setName("Intake");

    extensionState = ExtensionState.STOW;
    rollerState = RollerState.STOP;

    this.triggerSupplier = triggerSupplier;

    leftIntakeRoller =
        new GenericRoller(
            leftIntakeRollerIO,
            this,
            V3_HorseIntakeConstants.LEFT_INTAKE_ROLLER_CONSTANTS,
            "Left Intake Roller",
            V3_HorseIntakeConstants.INTAKE_ROLLER_STATES.get(rollerState));
    rightIntakeRoller =
        new GenericRoller(
            rightIntakeRollerIO,
            this,
            V3_HorseIntakeConstants.RIGHT_INTAKE_ROLLER_CONSTANTS,
            "Right Intake Roller",
            V3_HorseIntakeConstants.INTAKE_ROLLER_STATES.get(rollerState));
    kickerRoller =
        new GenericRoller(
            kickerRollerIO,
            this,
            V3_HorseIntakeConstants.KICKER_ROLLER_CONSTANTS,
            "Kicker",
            V3_HorseIntakeConstants.KICKER_ROLLER_STATES.get(rollerState));

    leftExtension =
        new Extension(
            V3_HorseIntakeConstants.LEFT_EXTENSION_CONSTANTS,
            this,
            0,
            leftExtensionIO,
            V3_HorseIntakeConstants.LEFT_EXTENSION_STATES.get(extensionState));
    rightExtension =
        new Extension(
            V3_HorseIntakeConstants.RIGHT_EXTENSION_CONSTANTS,
            this,
            0,
            rightExtensionIO,
            V3_HorseIntakeConstants.RIGHT_EXTENSION_STATES.get(extensionState));

    leftManualPositionGoal =
        new Setpoint<>(
            Meters.of(V3_HorseIntakeConstants.EXTENSION_STOW_POSITION),
            Meters.of(0.01),
            Meters.of(V3_HorseIntakeConstants.MIN_EXTENSION),
            Meters.of(V3_HorseIntakeConstants.MAX_EXTENSION));

    rightManualPositionGoal =
        new Setpoint<>(
            Meters.of(V3_HorseIntakeConstants.EXTENSION_STOW_POSITION),
            Meters.of(0.01),
            Meters.of(V3_HorseIntakeConstants.MIN_EXTENSION),
            Meters.of(V3_HorseIntakeConstants.MAX_EXTENSION));

    extensionPosition = Meters.zero();

    manualReachedStow = false;
    leftCompliant = false;
    rightCompliant = false;

    leftGainSlot = GainSlot.ZERO;
    rightGainSlot = GainSlot.ZERO;

    previousExtensionState = extensionState;
  }

  @Trace
  @Override
  public void periodic() {
    boolean triggerAtStow =
        manualAgitatePosition().in(Meters)
            <= (V3_HorseIntakeConstants.EXTENSION_STOW_POSITION
                + V3_HorseIntakeConstants.AGITATE_AUTO_STOW_THRESHOLD);

    manualReachedStow =
        (extensionState == ExtensionState.MANUAL_EXTEND) && (manualReachedStow || triggerAtStow);

    if (extensionState == ExtensionState.MANUAL_EXTEND
        && triggerAtStow
        && isNearStow(V3_HorseIntakeConstants.AGITATE_AUTO_STOW_THRESHOLD)) {
      extensionState = ExtensionState.STOW;
    }

    switch (extensionState) {
      case INTAKE, STOW:
        leftExtension.setPositionGoal(
            V3_HorseIntakeConstants.LEFT_EXTENSION_STATES.get(extensionState));
        rightExtension.setPositionGoal(
            V3_HorseIntakeConstants.RIGHT_EXTENSION_STATES.get(extensionState));
        break;
      case MANUAL_EXTEND:
        leftManualPositionGoal.setSetpoint(manualAgitatePosition());
        rightManualPositionGoal.setSetpoint(manualAgitatePosition());

        leftExtension.setPositionGoal(leftManualPositionGoal);
        rightExtension.setPositionGoal(leftManualPositionGoal);
        break;
      case SLOW_MOVE, SQUISH, STOP, SYS_ID, ZERO:
        // open loop control, we don't need to do anything
        break;
    }

    updateGainSlots();

    leftIntakeRoller.setVoltageGoal(V3_HorseIntakeConstants.INTAKE_ROLLER_STATES.get(rollerState));
    rightIntakeRoller.setVoltageGoal(V3_HorseIntakeConstants.INTAKE_ROLLER_STATES.get(rollerState));
    kickerRoller.setVoltageGoal(V3_HorseIntakeConstants.KICKER_ROLLER_STATES.get(rollerState));

    leftIntakeRoller.periodic();
    rightIntakeRoller.periodic();
    rightExtension.periodic();
    leftExtension.periodic();
    kickerRoller.periodic();

    extensionPosition = rightExtension.getExtensionPosition();

    Logger.recordOutput(
        "Intake/Roller/State",
        rollerState.toString()); // cause u need to give it a state not just an enum
    Logger.recordOutput("Intake/Extension/State", extensionState.toString());
    Logger.recordOutput("Intake/Extension/Trigger", triggerSupplier.getAsDouble());
  }

  private Distance manualAgitatePosition() {
    return Meters.of(
        triggerSupplier.getAsDouble()
                * (V3_HorseIntakeConstants.EXTENSION_INTAKE_POSITION
                    - V3_HorseIntakeConstants.EXTENSION_STOW_POSITION)
            + V3_HorseIntakeConstants.EXTENSION_STOW_POSITION);
  }

  private boolean isNearStow(double tolerance) {
    double threshold = V3_HorseIntakeConstants.EXTENSION_STOW_POSITION + tolerance;

    return ((leftExtension.getExtensionPosition().in(Meters) <= threshold)
        && (rightExtension.getExtensionPosition().in(Meters) <= threshold));
  }

  private void updateGainSlots() {
    boolean holdingGoal = (extensionState == ExtensionState.INTAKE);

    if (!holdingGoal || extensionState != previousExtensionState) {
      leftCompliant = false;
      rightCompliant = false;
    } else {
      leftCompliant |=
          leftExtension
              .atPositionGoal(); // leftCompliant = leftComplaint || leftExtension.getPositionGoal
      rightCompliant |= rightExtension.atPositionGoal();
    }

    previousExtensionState = extensionState;

    if (holdingGoal
        || extensionState == ExtensionState.STOW
        || extensionState == ExtensionState.MANUAL_EXTEND) {
      GainSlot leftSlot = leftCompliant ? GainSlot.ONE : GainSlot.ZERO;
      GainSlot rightSlot = rightCompliant ? GainSlot.ONE : GainSlot.ZERO;

      if (leftSlot != leftGainSlot) {
        leftExtension.setGainSlot(leftSlot);
        leftGainSlot = leftSlot;
      }

      if (rightSlot != rightGainSlot) {
        rightExtension.setGainSlot(rightSlot);
        rightGainSlot = rightSlot;
      }
    }

    Logger.recordOutput("Intake/Extension/Left Compliant", leftCompliant);
    Logger.recordOutput("Intake/Extension/Right Compliant", rightCompliant);
  }

  public Command stopRollers() {
    return Commands.runOnce(() -> rollerState = RollerState.STOP);
  }

  public Command intakeRollers() {
    return Commands.runOnce(() -> rollerState = RollerState.INTAKE);
  }

  public Command extakeRollers() {
    return Commands.runOnce(() -> rollerState = RollerState.EXTAKE);
  }

  public Command extend() {
    return Commands.runOnce(() -> extensionState = ExtensionState.INTAKE);
  }

  public Command setManualExtend() {
    return Commands.runOnce(() -> extensionState = ExtensionState.MANUAL_EXTEND);
  }

  public Command retract() {
    return Commands.runOnce(() -> extensionState = ExtensionState.STOW);
  }

  public Command setExtensionState(ExtensionState state) {
    return Commands.runOnce(() -> extensionState = state);
  }

  public Command setRollerState(RollerState state) {
    return Commands.runOnce(() -> rollerState = state);
  }

  public Command releaseManualExtend() {
    return Commands.runOnce(
        () -> {
          extensionState = manualReachedStow ? ExtensionState.STOW : ExtensionState.INTAKE;
        });
  }

  public Command stopExtensions() {
    return Commands.runOnce(
        () -> {
          extensionState = ExtensionState.STOP;
          leftExtension.setVoltageGoal(Volts.of(0));
          rightExtension.setVoltageGoal(Volts.of(0));
        });
  }

  public Command resetIntakeZeroPosition() {
    return Commands.runOnce(
            () -> {
              leftExtension.setPosition(Meters.of(V3_HorseIntakeConstants.MIN_EXTENSION));
              rightExtension.setPosition(Meters.of(V3_HorseIntakeConstants.MIN_EXTENSION));
              extensionState = ExtensionState.STOW;
            })
        .ignoringDisable(true);
  }

  public Command slowMoveIn() {
    return Commands.runOnce(
            () -> {
              extensionState =
                  ExtensionState.SLOW_MOVE; // setting it to a diff state so it doesnt ignore
              leftExtension.setVoltageGoal(
                  V3_HorseIntakeConstants.EXTENSION_SLOW_MOVE_VOLTAGE.times(-1));
              rightExtension.setVoltageGoal(
                  V3_HorseIntakeConstants.EXTENSION_SLOW_MOVE_VOLTAGE.times(-1));
            })
        .finallyDo(
            interrupted -> {
              if (interrupted) extensionState = ExtensionState.STOP;
              leftExtension.setVoltageGoal(Volts.zero());
              rightExtension.setVoltageGoal(Volts.zero());
            });
  }

  public Command slowMoveOut() {
    return Commands.runOnce(
        () -> {
          extensionState =
              ExtensionState.SLOW_MOVE; // setting it to a diff state so it doesnt ignore
          leftExtension.setVoltageGoal(V3_HorseIntakeConstants.EXTENSION_SLOW_MOVE_VOLTAGE);
          rightExtension.setVoltageGoal(V3_HorseIntakeConstants.EXTENSION_SLOW_MOVE_VOLTAGE);
        });
  }

  public Command zeroExtensions() {
    return Commands.sequence(
        Commands.runOnce(() -> rollerState = RollerState.STOP),
        Commands.runOnce(() -> extensionState = ExtensionState.ZERO),
        Commands.parallel(leftExtension.resetExtensionZero(), rightExtension.resetExtensionZero()));
  }

  public Command squishIntake() { // agitate while shooting automatically
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  if (extensionState == ExtensionState.MANUAL_EXTEND
                      || extensionState == ExtensionState.STOW) {
                    return;
                  }

                  extensionState = ExtensionState.SQUISH;

                  leftExtension.setVoltageGoal(V3_HorseIntakeConstants.SQUISH_VOLTAGE.get());
                  rightExtension.setVoltageGoal(V3_HorseIntakeConstants.SQUISH_VOLTAGE.get());
                }),
            Commands.waitUntil(() -> extensionState != ExtensionState.SQUISH || isNearStow(0.01))
                .withTimeout(5))
        .finallyDo(
            () -> {
              if (extensionState == ExtensionState.SQUISH) {
                extensionState = ExtensionState.STOP;

                leftExtension.setVoltageGoal(Volts.zero());
                rightExtension.setVoltageGoal(Volts.zero());

                extensionState = ExtensionState.STOW;
              }
            });
  }

  public Command sysID() {
    return (setExtensionState(ExtensionState.SYS_ID))
        .andThen(Extension.runSysIdRoutine(this, leftExtension, rightExtension));
  }
}
