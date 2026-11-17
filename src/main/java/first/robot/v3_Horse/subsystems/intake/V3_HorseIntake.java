package first.robot.v3_Horse.subsystems.intake;

import static org.wpilib.units.Units.Meters;

import java.util.function.DoubleSupplier;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.measure.Distance;

import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.subsystems.extension.Extension;
import edu.wpi.team190.gompeilib.subsystems.extension.ExtensionIO;
import edu.wpi.team190.gompeilib.subsystems.extension.ExtensionIOSim;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntakeConstants.ExtensionState;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntakeConstants.RollerState;
import lombok.Getter;

public class V3_HorseIntake extends SubsystemBase {
    @Getter
    private ExtensionState extensionState;
    @Getter
    private RollerState rollerState;

    private final GenericRoller rightIntakeRoller;
    private final GenericRoller leftIntakeRoller;

    public final Extension rightExtension;
    public final Extension leftExtension;

    private final GenericRoller kickerRoller;

    private DoubleSupplier triggerSupplier;

    private final Setpoint<DistanceUnit> leftManualPositionGoal;
    private final Setpoint<DistanceUnit> rightManualPositionGoal;

    @Getter
    private Distance extensionPosition;

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

        leftIntakeRoller = new GenericRoller(leftIntakeRollerIO, this, V3_HorseIntakeConstants.LEFT_INTAKE_ROLLER_CONSTANTS, "Left Intake Roller", V3_HorseIntakeConstants.INTAKE_ROLLER_STATES.get(rollerState));
        rightIntakeRoller = new GenericRoller(rightIntakeRollerIO, this, V3_HorseIntakeConstants.RIGHT_INTAKE_ROLLER_CONSTANTS, "Right Intake Roller", V3_HorseIntakeConstants.INTAKE_ROLLER_STATES.get(rollerState));
        kickerRoller = new GenericRoller(kickerRollerIO, this, V3_HorseIntakeConstants.KICKER_ROLLER_CONSTANTS, "Kicker", V3_HorseIntakeConstants.KICKER_ROLLER_STATES.get(rollerState));

        leftExtension = new Extension(V3_HorseIntakeConstants.LEFT_EXTENSION_CONSTANTS, this, 0, leftExtensionIO, V3_HorseIntakeConstants.LEFT_EXTENSION_STATES.get(extensionState));
        rightExtension = new Extension(V3_HorseIntakeConstants.RIGHT_EXTENSION_CONSTANTS, this, 0, rightExtensionIO, V3_HorseIntakeConstants.RIGHT_EXTENSION_STATES.get(extensionState));

        leftManualPositionGoal = new Setpoint<>(
            Meters.of(V3_HorseIntakeConstants.EXTENSION_STOW_POSITION),
            Meters.of(0.01),
            Meters.of(V3_HorseIntakeConstants.MIN_EXTENSION),
            Meters.of(V3_HorseIntakeConstants.MAX_EXTENSION)
        );

        rightManualPositionGoal = new Setpoint<>(
            Meters.of(V3_HorseIntakeConstants.EXTENSION_STOW_POSITION),
            Meters.of(0.01),
            Meters.of(V3_HorseIntakeConstants.MIN_EXTENSION),
            Meters.of(V3_HorseIntakeConstants.MAX_EXTENSION)
        );

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
            boolean triggerAtStow = manualAgitatePosition().in(Meters) <= (V3_HorseIntakeConstants.EXTENSION_STOW_POSITION + V3_HorseIntakeConstants.AGITATE_AUTO_STOW_THRESHOLD);

            manualReachedStow = (extensionState == ExtensionState.MANUAL_EXTEND) && (manualReachedStow || triggerAtStow);

            if (extensionState == ExtensionState.MANUAL_EXTEND && triggerAtStow && isNearStow(V3_HorseIntakeConstants.AGITATE_AUTO_STOW_THRESHOLD)) {
                extensionState = ExtensionState.STOW;
            }

            switch (extensionState) {
                case INTAKE, STOW:
                    leftExtension.setPositionGoal(V3_HorseIntakeConstants.LEFT_EXTENSION_STATES.get(extensionState));
                    rightExtension.setPositionGoal(V3_HorseIntakeConstants.RIGHT_EXTENSION_STATES.get(extensionState));
                    break;
                case MANUAL_EXTEND:
                    leftManualPositionGoal.setSetpoint(manualAgitatePosition());
                    rightManualPositionGoal.setSetpoint(manualAgitatePosition());
                    
                    leftExtension.setPositionGoal(leftManualPositionGoal);
                    rightExtension.setPositionGoal(leftManualPositionGoal);
                    break;
                case SLOW_MOVE, SQUISH, STOP, SYS_ID, ZERO:
                    //open loop control, we don't need to do anything
                    break;
            }
        }
        
    private Distance manualAgitatePosition() {
        return Meters.of(triggerSupplier.getAsDouble() * (V3_HorseIntakeConstants.EXTENSION_INTAKE_POSITION - V3_HorseIntakeConstants.EXTENSION_STOW_POSITION) + V3_HorseIntakeConstants.EXTENSION_STOW_POSITION);
    }

    private boolean isNearStow(double tolerance) {
        double threshold = V3_HorseIntakeConstants.EXTENSION_STOW_POSITION + tolerance;

        return ((leftExtension.getExtensionPosition().in(Meters) <= threshold) && (rightExtension.getExtensionPosition().in(Meters) <= threshold));
    }

    private void updateGainSlots() {
        boolean holdingGoal = (extensionState == ExtensionState.INTAKE);

            if (!holdingGoal || extensionState != previousExtensionState) {
                leftCompliant = false;
                rightCompliant = false;
            }

            else {
                leftCompliant |= leftExtension.atPositionGoal(); //leftCompliant = leftComplaint || leftExtension.getPositionGoal
                rightCompliant |= rightExtension.atPositionGoal();
            }
    }
}
