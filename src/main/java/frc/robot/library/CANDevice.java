package frc.robot.library;

import frc.robot.functions.io.xmlreader.Device;
import org.w3c.dom.Element;

public class CANDevice extends Device {

    private int mintCANID;

    public CANDevice(String name, int canid) {
        super(name);
        mintCANID = canid;
    }

    public CANDevice(Element element) {
        super(element);
        mintCANID = Integer.parseInt(getOrDefault(element, "ID", "0"));
    }

    public int getCANID() {
        return mintCANID;
    }

    public void setCANID(int mintCANID) {
        this.mintCANID = mintCANID;
    }

    @Override
    public String toString() {
        return super.toString() + "\nID: " + mintCANID;
    }
}

