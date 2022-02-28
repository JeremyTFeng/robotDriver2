package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.w3c.dom.Element;
import java.io.File;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

public class ParseXYFromXML {
    private static String TAG = "ParseXYFromXML";
    private Pose2d[] coordinates;
    private String[] ops;
    private String fileName;
    public ParseXYFromXML(String file_name) {
        fileName = file_name;
        parsePathXY();
    }
    public Pose2d[] getCoordinates() {
        return coordinates;
    }
    public String[] getOperations() {
        return ops;
    }
    private void parsePathXY() {
        String full_path = AppUtil.CONFIG_FILES_DIR + "/" + fileName;
        RobotLogger.dd(TAG, "path definition file: " + full_path);
        coordinates = null;
        ops = null;
        try {
            File inputFile = new File(AppUtil.CONFIG_FILES_DIR+"/"+fileName);
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            Document doc = dBuilder.parse(inputFile);
            doc.getDocumentElement().normalize();
            RobotLogger.dd(TAG, "Root element :" + doc.getDocumentElement().getNodeName());
            NodeList nList = doc.getElementsByTagName("movestep");
            RobotLogger.dd(TAG,"----------------------------");
            int step_num = nList.getLength();
            RobotLogger.dd(TAG, "elements :" + Integer.toString(step_num));

            coordinates = new Pose2d[step_num];
            ops = new String[step_num];

            for (int temp = 0; temp < nList.getLength(); temp++) {
                Node nNode = nList.item(temp);
                //RobotLogger.dd(TAG, "Current Element :" + nNode.getNodeName());

                if (nNode.getNodeType() == Node.ELEMENT_NODE) {
                    double x, y, h;
                    String op;
                    Element eElement = (Element) nNode;
                    RobotLogger.dd(TAG, "step info : "
                            + eElement.getAttribute("drive_type"));
                    x = new Double(eElement
                            .getElementsByTagName("x")
                            .item(0)
                            .getTextContent());
                    y = new Double(eElement
                            .getElementsByTagName("y")
                            .item(0)
                            .getTextContent());
                    h = new Double(eElement
                            .getElementsByTagName("h")
                            .item(0)
                            .getTextContent());
                    op = eElement.getElementsByTagName("o")
                            .item(0)
                            .getTextContent();
                    RobotLogger.dd(TAG,"step %d: x: %f, y: %f, h: %f, o: %s", temp, x, y , h, op.toString());
                    coordinates[temp] = new Pose2d(x, y, Math.toRadians(h));
                    ops[temp] = op.toString();
                }
            }
        } catch (Exception e) {
            RobotLogger.dd(TAG, "parsing failure for path XY file: " + full_path);
            e.printStackTrace();
        }
    }

}

