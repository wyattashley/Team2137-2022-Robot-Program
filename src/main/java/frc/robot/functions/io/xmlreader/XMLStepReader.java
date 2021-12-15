package frc.robot.functions.io.xmlreader;

import edu.wpi.first.wpiutil.math.Pair;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import java.io.File;
import java.util.*;
import java.util.function.Consumer;

public class XMLStepReader {
    private final File stepFile;

    private final List<Step> steps = new ArrayList<>();

    public XMLStepReader(String dir, String fileName) {
        this.stepFile = new File(dir + fileName);
        DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();

        Document doc = null;

        try { 
            DocumentBuilder db = dbf.newDocumentBuilder();  
            doc = db.parse(this.stepFile);  
            doc.getDocumentElement().normalize();  
        } catch(Exception e) {
            e.printStackTrace();
        }

        NodeList tmpStep = doc.getElementsByTagName("Step");

        for (int i = 0; i < tmpStep.getLength(); i++) {
            this.steps.add(parseSteps(tmpStep.item(i)));
        }
    }

    public List<Step> getSteps() {
        return this.steps;
    }

    public Step parseSteps(Node stepNode) {
        Element element = (Element) stepNode;
        NodeList parmNodeList = element.getElementsByTagName("parm");
        String[] tmpParms = new String[parmNodeList.getLength()];

        for (int i = 0; i < parmNodeList.getLength(); i++) {
            Element id = (Element) parmNodeList.item(0);
            tmpParms[Integer.parseInt(id.getAttribute("id"))] = parmNodeList.item(i).getTextContent();
        }

        Step returner = new Step(tmpParms);

        for (Step.StepValues a : Step.StepValues.values()) {
            NodeList node = element.getElementsByTagName(a.toString().toLowerCase());
            if(node.getLength() > 0)
                returner.setValue(new Pair<>(a, node.item(0).getTextContent()));
        }
        return returner;
    }

}