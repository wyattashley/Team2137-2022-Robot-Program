package frc.robot.functions.io.xmlreader;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.library.CANDevice;
import frc.robot.library.Constants.PIDSlotIDOutOfRangeException;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

public class Motor extends CANDevice {

    public static final int NUMBEROFPIDSLOTS = 5;

    private final MotorTypes type;
    private final boolean inverted;
    private final int currentLimit;
    private final double gearRatio;
    private final double rampRate;
    private String[] parms;
    private PID[] pidValues;

    /**
     * Create a new Motor Object for debug and better storage
     *
     * @param _name
     * @param _id
     * @param _type
     * @param _invert
     * @param _currentLimit
     * @param _gearRatio
     * @param _parms
     */
    public Motor(String _name, int _id, MotorTypes _type, boolean _invert, int _currentLimit, double _gearRatio, double _ramp, String... _parms) {
        super(_name, _id);
        this.type = _type;
        this.inverted = _invert;
        this.parms = _parms;
        this.currentLimit = _currentLimit;
        this.gearRatio = _gearRatio;
        this.rampRate = _ramp;
    }

    public Motor(String _name, int _id, MotorTypes _type, boolean _invert, int _currentLimit, double _gearRatio, double _ramp, PID[] pid, String... _parms) {
        this(_name, _id, _type, _invert, _currentLimit, _gearRatio, _ramp, _parms);
        this.pidValues = pid;
    }

    public Motor(CANDevice _device, MotorTypes _type, boolean _invert, int _currentLimit, double _gearRatio, double _ramp, PID[] pid, String... _parms) {
        this(_device.getName(), _device.getCANID(), _type, _invert, _currentLimit, _gearRatio, _ramp, _parms);
        this.pidValues = pid;
    }

    public Motor(Element element) {
        super(element);
        this.type = MotorTypes.valueOf(getOrDefault(element, "Type", "NEO").toUpperCase());
        this.inverted = Boolean.parseBoolean(getOrDefault(element, "Inverted", "false").toLowerCase());
        this.currentLimit = Integer.parseInt(getOrDefault(element, "CurrentLimit", "-1"));
        this.gearRatio = Double.parseDouble(getOrDefault(element, "GearRatio", "1"));
        this.rampRate = Double.parseDouble(getOrDefault(element, "RampRate", "0"));

        NodeList tmpList = element.getElementsByTagName("PID");
        pidValues = new PID[NUMBEROFPIDSLOTS];

        try {
            for (int i = 0; i < tmpList.getLength(); i++) {
                String value = ((Element) tmpList.item(i)).getAttribute("Slot");
                int slotNumber = 0;
                if(!value.equals(""))
                    slotNumber = Integer.parseInt(value);

                if (slotNumber >= NUMBEROFPIDSLOTS) {
                    throw new PIDSlotIDOutOfRangeException(getName());
                } else {
                    this.pidValues[slotNumber] = new PID((Element) tmpList.item(i));
                }

            }
        } catch (PIDSlotIDOutOfRangeException e) {
            e.printStackTrace();
        }
    }

    public double getGearRatio() {
        return this.gearRatio;
    }

    public MotorTypes getMotorType() {
        return this.type;
    }

    public String[] getParms() {
        return this.parms;
    }

    public String getParm(int i) {
        return this.parms[i];
    }

    public Double getParmDouble(int i) {
        return Double.parseDouble(this.parms[1]);
    }

    public boolean inverted() {
        return this.inverted;
    }

    public int getCurrentLimit() {
        return this.currentLimit;
    }

    public PID getPID(int slotID) {
        return this.pidValues[slotID];
    }

    public double getRampRate() {
        return rampRate;
    }

    @Override
    public String toString() {
        return super.toString() + "\nType: " + type.toString() + "\nInverted: " + inverted + "\nCurrent Limit: " + currentLimit
                + "\nGear Ratio: " + gearRatio + "\nRamp Rate: " + rampRate + "\nPID: " + pidValues.toString();
    }

    public enum MotorTypes {
        NEO(MotorType.kBrushless, "NEO"),
        NEO550(MotorType.kBrushless, "NEO550"),
        FALCON(MotorType.kBrushless, "FALCON"),
        BAG(MotorType.kBrushed, "BAG"),
        CIM(MotorType.kBrushed, "CIM"),
        WINDOW(MotorType.kBrushed, "WINDOW");

        MotorType type;
        String value;

        MotorTypes(MotorType revType, String name) {
            type = revType;
            value = name;
        }

        public MotorType getREVType() {
            return this.type;
        }

        @Override
        public String toString() {
            return value;
        }
    }

    public enum ControllerType {
        SPARK("Spark"),
        TALONFX("TalonFX"),
        TALON("Talon");

        String value;

        ControllerType(String name) {
            value = name;
        }

        public static ControllerType getControllerFromMotor(MotorTypes type) {
            switch (type) {
                case NEO:
                case NEO550:
                    return SPARK;
                case FALCON:
                    return TALONFX;
                default:
                    return TALON;
            }
        }

        @Override
        public String toString() {
            return value;
        }
    }
}