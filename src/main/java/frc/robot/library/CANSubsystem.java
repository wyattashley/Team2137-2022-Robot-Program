package frc.robot.library;

import frc.robot.functions.io.FileLogger;
import frc.robot.functions.io.xmlreader.*;
import org.w3c.dom.Element;

import frc.robot.library.Constants.MissingDeviceException;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class CANSubsystem {
    public static Device createDevice(Element element, String name) {
        switch (name) {
            case "Motor":
                return new Motor(element);
            case "Encoder":
                return new Encoder(element);
            case "Gyro":
                return new Gyro(element);
            case "PID":
                return new PID(element);
            default:
                return null;
        }
    }

    private final String type;
    private final String name;
    private final HashMap<String, Device> devices = new HashMap<>();
    private final HashMap<String, CANSubsystem> childSubsystem = new HashMap<>();

    public CANSubsystem(Element element) {
        type = element.getTagName();

        String value = element.getAttribute("name");
        if(!value.equals(""))
            name = value;
        else
            name = element.getTagName();
    }

    public void addDevice(Device _device) {
        devices.put(_device.getName(), _device);
    }

    public HashMap<String, CANSubsystem> getChildSubsystems() {
        return childSubsystem;
    }

    public ArrayList<CANSubsystem> getSubsystemByType(String type) {
        ArrayList<CANSubsystem> returningList = new ArrayList<>();

        for (Map.Entry<String, CANSubsystem> sub : childSubsystem.entrySet()) {
            if (sub.getValue().getSubsystemType().equals(type)) {
                returningList.add(sub.getValue());
            }
        }

        return returningList;
    }

    public String getSubsystemType() {
        return type;
    }

    public String getName() {
        return name;
    }

    public Device getDevice(String name) {
        return devices.get(name);
    }

    public CANDevice getCANDevice(String name) {
        return (CANDevice) devices.get(name);
    }

    public CANSubsystem getRequiredSubsystem(String name, String type) throws MissingDeviceException {
        if(childSubsystem.containsKey(name) && childSubsystem.get(name).getSubsystemType().equals(type))
            return childSubsystem.get(name);
        else
            throw new MissingDeviceException(this, name);
    }

    public Device getRequiredDevice(String name) throws MissingDeviceException {
        if(devices.containsKey(name))
            return devices.get(name);
        else
            throw new MissingDeviceException(this, name);
    }

    public void addSubsystem(CANSubsystem subsystem) {
        childSubsystem.put(subsystem.getName(), subsystem);
    }

    public void printHardwareMap(FileLogger fileLogger) {
        printHardwareMap(0, fileLogger);
    }

    private void printHardwareMap(int depth, FileLogger fileLogger) {
        for(int i = 0; i < depth; i++)
            fileLogger.rawWrite("\t");

        fileLogger.rawWrite(getName() + " (" + getSubsystemType() + " Subsystem)\n");

        for(Device device : devices.values()) {
            for(int a = -1; a < depth; a++)
                fileLogger.rawWrite("\t");

            fileLogger.rawWrite(device.getName() + " (Device)\n");
        }

        for(Map.Entry<String, CANSubsystem> subsystem : childSubsystem.entrySet()) {
            subsystem.getValue().printHardwareMap(depth + 1, fileLogger);
        }

        if(depth == 0)
            printDevicesOnly(fileLogger);
    }

    public void printDevicesOnly(FileLogger fileLogger) {
        for(Device device : devices.values()) {
            fileLogger.writeLine(device.toString() + "\n");
        }

        for(Map.Entry<String, CANSubsystem> subsystem : childSubsystem.entrySet()) {
            subsystem.getValue().printDevicesOnly(fileLogger);
        }
    }
}
