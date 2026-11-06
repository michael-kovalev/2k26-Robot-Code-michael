package first.robot.v3_Horse.subsystems.rollerfloor;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Voltage;

import static org.wpilib.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import first.robot.v3_Horse.subsystems.rollerfloor.V3_HorseRollerFloorConstants.RollerFloorState;
import lombok.Getter;

public class V3_HorseRollerFloor extends SubsystemBase {
    private GenericRoller rollerFloor;
    
    @Getter private RollerFloorState rollerFloorGoal;

    private final Setpoint<VoltageUnit> rollerFloorSetpoint, rollerFloorOverrideSetpoint;

    private final Setpoint<VoltageUnit> rollerFloorStopSetpoint;

    public V3_HorseRollerFloor(GenericRollerIO rollerFloorIO) {
        rollerFloorGoal = RollerFloorState.STOP;

        rollerFloorSetpoint = new Setpoint<VoltageUnit>(V3_HorseRollerFloorConstants.ROLLER_FLOOR_RUN_VOLTAGE, V3_HorseRollerFloorConstants.ROLLER_FLOOR_CONSTANTS.voltageOffsetStep, Volts.of(-12.0), Volts.of(12.0));
        rollerFloorOverrideSetpoint = new Setpoint<VoltageUnit>(V3_HorseRollerFloorConstants.ROLLER_FLOOR_OVERRIDE_VOLTAGE, V3_HorseRollerFloorConstants.ROLLER_FLOOR_CONSTANTS.voltageOffsetStep, Volts.of(-12.0), Volts.of(12.0));
        rollerFloorStopSetpoint = new Setpoint<VoltageUnit>(V3_HorseRollerFloorConstants.ROLLER_FLOOR_RUN_VOLTAGE, V3_HorseRollerFloorConstants.ROLLER_FLOOR_CONSTANTS.voltageOffsetStep, Volts.of(0), Volts.of(0));

        rollerFloor = new GenericRoller(rollerFloorIO, this, V3_HorseRollerFloorConstants.ROLLER_FLOOR_CONSTANTS, "Floor", rollerFloorSetpoint);
    }

    @Override
    public void periodic() {
        switch (rollerFloorGoal) {
            case RUN:
                runRollerFloor(rollerFloorOverrideSetpoint);
                break;
            case OVERRIDE: 
                runRollerFloor(rollerFloorOverrideSetpoint);
                break;
            case STOP:
                stopRollerFloor();
                break;
        }
        
        rollerFloor.periodic();

        Logger.recordOutput("Elastic/Hopper/RollerFloor/Voltage Magnitude", String.format("%.1f", Math.abs(rollerFloor.getVoltageGoal().getSetpoint().in(Volts))));

        Logger.recordOutput("Elastic/Hopper/RollerFloor/Voltage Offset", String.format("%.1f", rollerFloor.getVoltageGoal().getOffset().in(Volts)));
    }

    public Command setState(RollerFloorState state) {
        return Commands.runOnce(() -> rollerFloorGoal = state);
    }
    
    public void setRollerFloorVoltage(Voltage volts) {
        rollerFloorSetpoint.setSetpoint(volts);
    }

    public void setRollerFloorOverrideVoltage(Voltage volts) {
        rollerFloorOverrideSetpoint.setSetpoint(volts);
    }

    public void runRollerFloor(Setpoint<VoltageUnit> voltageSetpoint) {
        rollerFloor.setVoltageGoal(voltageSetpoint);
    }

    public void stopRollerFloor() {
        rollerFloor.setVoltageGoal(rollerFloorStopSetpoint);
    }

    public Command incrementRollerFloorVelocity() {
        return Commands.either(Commands.runOnce(rollerFloorSetpoint::increment), Commands.either(
            Commands.runOnce(rollerFloorOverrideSetpoint::increment), Commands.runOnce(() -> stopRollerFloor()),
        () -> rollerFloorGoal.equals(RollerFloorState.OVERRIDE)),
        () -> rollerFloorGoal.equals(RollerFloorState.RUN));
    }

    public Command decrementRollerFloorVelocity() {
        return Commands.either(Commands.runOnce(rollerFloorSetpoint::decrement), Commands.either(
            Commands.runOnce(rollerFloorOverrideSetpoint::decrement), Commands.runOnce(() -> stopRollerFloor()),
        () -> rollerFloorGoal.equals(RollerFloorState.OVERRIDE)),
        () -> rollerFloorGoal.equals(RollerFloorState.RUN));
    }
}
