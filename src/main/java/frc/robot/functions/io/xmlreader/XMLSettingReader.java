package frc.robot.functions.io.xmlreader;

import frc.robot.functions.io.FileLogger;
import frc.robot.library.CANSubsystem;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import java.io.File;
import java.util.HashMap;

public class XMLSettingReader {

    private final File settingFile;
    private HashMap<String, String> settings;
    private CANSubsystem robotCANSubsystem;

    private final FileLogger logger;

    public XMLSettingReader(String dir, String fileName, FileLogger _logger) {
        this.logger = _logger;

        this.settingFile = new File(dir + fileName);
        DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();

        Document doc;

        try {
            DocumentBuilder db = dbf.newDocumentBuilder();
            doc = db.parse(this.settingFile);
            doc.getDocumentElement().normalize();
        } catch(Exception e) {
            e.printStackTrace();
            return;
        }

        logger.writeEvent(9, "Searching for devices in xml file");

        Node hardware = doc.getElementsByTagName("Hardware").item(0);
        robotCANSubsystem = new CANSubsystem((Element) hardware);
        findDevices(hardware, robotCANSubsystem, 0);

        NodeList settingNode = doc.getElementsByTagName("Settings");

        settings = new HashMap<>();

        if(settingNode.getLength() > 0) {
            NodeList values = settingNode.item(0).getChildNodes();
            for (int i = 0; i < values.getLength(); i++) {
                Node tmp = values.item(i);
                if(tmp.getNodeType() == Node.ELEMENT_NODE) {
                    Element element = (Element) tmp;
                    settings.put(element.getTagName(), element.getTextContent());
                }
            }
        }
    }

    protected void findDevices(Node element, CANSubsystem subSystem, int depth) {
        NodeList list = element.getChildNodes();
        if (list.getLength() > 1) {
            for(int i = 0; i < list.getLength(); i++) {
                if(list.item(i).getNodeType() == Node.ELEMENT_NODE) {
                    Element tmp = (Element) list.item(i);

                    Device tmpDevice = CANSubsystem.createDevice(tmp, tmp.getTagName());
                    if(tmpDevice == null) {
                        CANSubsystem tmpSubSystem = new CANSubsystem(tmp);
                        subSystem.addSubsystem(tmpSubSystem);
                        findDevices(tmp, tmpSubSystem, depth + 1);
                    } else {
                        subSystem.addDevice(tmpDevice);
                    }
                }
            }
        }
    }

    public CANSubsystem getRobotCANSubsystem() {
        return robotCANSubsystem;
    }

    public double getSetting(String str, double defaultVal) {
        return Double.parseDouble(getSetting(this.settings.get(str), String.valueOf(defaultVal)));
    }

    public String getSetting(String str, String defaultVal) {
        return this.settings.getOrDefault(str, defaultVal);
    }
}