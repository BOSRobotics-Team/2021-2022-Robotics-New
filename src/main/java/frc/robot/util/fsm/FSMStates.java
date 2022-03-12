package frc.robot.util.fsm;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import javax.xml.parsers.*;
import org.w3c.dom.*;
import org.xml.sax.SAXException;

/* Class that represents the FiniteStateMachine<br> */
public class FSMStates<T extends Enum<T>> {
  private final Class<T> m_clazz;
  private ArrayList<FSMState<T>> m_FSMStates = new ArrayList<FSMState<T>>();
  private FSMState<T> m_currentState;

  /**
   * This constructor allows to create a FiniteStateMachine with the states from an Enum <br>
   *
   * @param cl Specifies the Enum states class
   */
  public FSMStates(Class<T> cl) {
    this.m_clazz = cl;
    for (T state : cl.getEnumConstants()) {
      this.m_FSMStates.add(new FSMState<T>(state));
    }
  }

  /**
   * This constructor creates a FiniteStateMachine from an XML configuration file<br>
   *
   * @param cl Specifies the Enum states class
   * @param configFileName XML Configuration file path
   * @param extFile If external, open with File otherwise open as Resource
   * @throws ParserConfigurationException
   * @throws SAXException
   * @throws IOException
   */
  public FSMStates(Class<T> cl, String configFileName, boolean extFile)
      throws ParserConfigurationException, SAXException, IOException {
    this.m_clazz = cl;
    if (!configFileName.isBlank()) {
      DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
      DocumentBuilder builder = factory.newDocumentBuilder();
      if (extFile) {
        this.initStates(builder.parse(new File(configFileName)));
      } else {
        this.initStates(
            builder.parse(this.getClass().getClassLoader().getResourceAsStream(configFileName)));
      }
    }
  }

  /**
   * This constructor allows to create a FiniteStateMachine from an InputStream<br>
   *
   * @param cl Specifies the Enum states class
   * @param configStream InputStream of a XML configuration file.
   * @throws ParserConfigurationException
   * @throws SAXException
   * @throws IOException
   */
  public FSMStates(Class<T> cl, InputStream configStream)
      throws ParserConfigurationException, SAXException, IOException {
    this.m_clazz = cl;
    DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
    DocumentBuilder builder = factory.newDocumentBuilder();
    this.initStates(builder.parse(configStream));
  }

  /**
   * Private method that will read states from a configuration file or stream
   *
   * @param doc XML Document from a configuration file or stream.
   */
  private void initStates(Document doc) {
    doc.getDocumentElement().normalize();
    NodeList nodeList = doc.getElementsByTagName(FSMState.kStateTag);
    for (int idx = 0; idx < nodeList.getLength(); idx++) {
      Node nNode = nodeList.item(idx);
      if (nNode.getNodeType() == Node.ELEMENT_NODE) {
        this.m_FSMStates.add(
            new FSMState<T>(
                Enum.valueOf(this.m_clazz, ((Element) nNode).getAttribute(FSMState.kIdTag)),
                this.getTransitionInfo(nNode)));
      }
    }
    this.setCurrentState(this.m_FSMStates.get(0));
  }

  /**
   * Method that sets the current state of the FiniteStateMachine <br>
   *
   * @param state The state enum to set
   */
  public void setCurrentState(T state) {
    for (FSMState<T> _s : this.m_FSMStates) {
      if (_s.getState().equals(state)) {
        this.setCurrentState(_s);
        return;
      }
    }
  }

  /**
   * Method that sets the current state of the FiniteStateMachine <br>
   *
   * @param state The FSMState object to set
   */
  public void setCurrentState(FSMState<T> state) {
    this.m_currentState = state;
  }

  /**
   * Method that gets the current state of the FiniteStateMachine <br>
   *
   * @return Returns a FSMState object
   */
  public FSMState<T> getCurrentState() {
    return this.m_currentState;
  }

  /**
   * Method that returns the list a FSMStates configured for this FiniteStateMachine. <br>
   *
   * @return Returns the list of FSMStates
   */
  public List<FSMState<T>> getAllStates() {
    return this.m_FSMStates;
  }

  /**
   * Method that adds a message and transition to a state. <br>
   *
   * @param state The state to add and message and transition to.
   * @param message The message string
   * @param action The transition action and next state
   */
  public void addMessage(T state, String message, FSMTransition<T> action) {
    for (FSMState<T> _s : this.m_FSMStates) {
      if (_s.getState().equals(state)) {
        _s.addMessage(message, action);
      }
    }
  }

  /**
   * Method that adds a message and transition to a state. <br>
   *
   * @param state The state to add and message and transition to.
   * @param message The message string
   * @param action The transition action
   * @param nextState The next state after tranition
   */
  public void addMessage(T state, String message, String action, T nextState) {
    this.addMessage(state, message, new FSMTransition<T>(nextState, action));
  }

  /**
   * Get a list of states from the XML configuration doc
   *
   * @return A List of states
   */
  public List<T> getStatesList(Document doc) {
    ArrayList<T> retVal = new ArrayList<T>();

    NodeList nodeList = doc.getElementsByTagName(FSMState.kStateTag);
    for (int idx = 0; idx < nodeList.getLength(); idx++) {
      Node nNode = nodeList.item(idx);
      if (nNode.getNodeType() == Node.ELEMENT_NODE) {
        retVal.add(Enum.valueOf(this.m_clazz, ((Element) nNode).getAttribute(FSMState.kIdTag)));
      }
    }
    return retVal;
  }

  /**
   * Find a specific state by ID from the XML configuration doc
   *
   * @param stateID The state ID to find
   * @return Returns the Element object if found or null
   */
  public Element findStateElement(Document doc, T stateID) {
    NodeList nodeList = doc.getElementsByTagName(FSMState.kStateTag);
    for (int idx = 0; idx < nodeList.getLength(); idx++) {
      Node node = nodeList.item(idx);
      if (node.getNodeType() == Node.ELEMENT_NODE) {
        if (((Element) node).getAttribute(FSMState.kIdTag).equals(stateID.toString()))
          return (Element) node;
      }
    }
    return null;
  }

  /**
   * Return a FMSTransitionMap from attributes of the given node in an XML configuration doc
   *
   * @param node The state node from the XML Document
   * @return A HashMap of transition attributes
   */
  public HashMap<String, FSMTransition<T>> getTransitionInfo(Node node) {
    HashMap<String, FSMTransition<T>> retVal = new HashMap<String, FSMTransition<T>>();

    if (null != node) {
      NodeList childNodes = node.getChildNodes();
      for (int idx = 0; idx < childNodes.getLength(); idx++) {
        Node childNode = childNodes.item(idx);
        if (childNode.getNodeType() == Node.ELEMENT_NODE) {
          retVal.put(
              ((Element) childNode).getAttribute(FSMState.kIdTag),
              new FSMTransition<T>(
                  Enum.valueOf(
                      this.m_clazz, ((Element) childNode).getAttribute(FSMState.kNextStateTag)),
                  ((Element) childNode).getAttribute(FSMState.kActionTag)));
        }
      }
    }
    return retVal;
  }
}
