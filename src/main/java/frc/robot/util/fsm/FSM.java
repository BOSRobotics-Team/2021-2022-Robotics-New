package frc.robot.util.fsm;

import java.io.IOException;
import java.io.InputStream;
import java.util.function.Consumer;
import javax.xml.parsers.ParserConfigurationException;
import org.xml.sax.SAXException;

/**
 * Class to allow creation of the FiniteStateMachine<br>
 * This class also allows developers to specify an XML Configuration File or inputstream of an XML
 * Configuration File.
 *
 * <p>Configuration file's format is as follows:
 *
 * <pre>{@code
 * <?xml version="1.0" encoding="UTF-8"?>
 *
 * <!--
 *  Document   : config.xml
 *  Description:
 *      File specifies states and transition of a FiniteStateMachine.
 *      This is an example file for Enum MoveStates &#123; START, INTERMEDIATE, STOP &#125;;
 *      FSM<MoveStates> _fsm = new FSM<MoveStates>(MoveStates.class,
 *                                  "config.xml",
 *                                  new Consumer<FSMAction<MoveStates>>(class::handler));
 * -->
 *
 * <FSM>
 * 	<STATE id="START" type="ID">
 * 		<MESSAGE id="MOVE" action="move" nextState="START"/>
 * 		<MESSAGE id="MOVELEFT" action="moveLeft" nextState="INTERMEDIATE"/>
 * 		<MESSAGE id="MOVERIGHT" action="moveRight" nextState="STOP"/>
 * 	</STATE>
 * 	<STATE id="INTERMEDIATE">
 * 		<MESSAGE id="MOVELEFT" action="moveLeft" nextState="STOP"/>
 * 		<MESSAGE id="MOVERIGHT" action="moveRight" nextState="STOP"/>
 * 	</STATE>
 * 	<STATE id="STOP">
 * 	</STATE>
 * </FSM>
 * }</pre>
 */
public class FSM<T extends Enum<T>> {
  /*
   * Finite State Machine and action handler
   */
  private FSMStates<T> m_FSM;
  private Consumer<FSMAction<T>> m_actionHandler;

  /**
   * Constructor that creates a FiniteStateMachine from a configuration filename<br>
   * and specified Action handler<br>
   *
   * @param configFileName : XML Configuration filename
   * @param action : An Action handler
   * @throws ParserConfigurationException
   * @throws SAXException
   * @throws IOException
   */
  public FSM(Class<T> cl, String configFileName, Consumer<FSMAction<T>> action)
      throws ParserConfigurationException, SAXException, IOException {
    this.m_FSM = new FSMStates<T>(cl, configFileName, !configFileName.isBlank());
    this.m_actionHandler = action;
  }

  /**
   * Constructor that creates a FiniteStateMachine from a specified Inputstream<br>
   * and specified Action handler<br>
   *
   * @param configStream Input Stream of the XML Configuration file.
   * @param action SAn Action handler
   * @throws ParserConfigurationException
   * @throws SAXException
   * @throws IOException
   */
  public FSM(Class<T> cl, InputStream configStream, Consumer<FSMAction<T>> action)
      throws ParserConfigurationException, SAXException, IOException {
    this.m_FSM = new FSMStates<T>(cl, configStream);
    this.m_actionHandler = action;
  }

  /**
   * Method to process an incoming Message and take an appropriate action<br>
   * and on successful execution of the action Transitions to the next state<br>
   * as per the transition map.<br>
   *
   * @param recvdMsgId Received Message Id
   * @return Returns true if transitions to a new state
   */
  public boolean processMessage(String recvdMsgId) {
    FSMTransition<T> transition = this.m_FSM.getCurrentState().getTransitionMap().get(recvdMsgId);
    if (null != transition) {
      for (FSMState<T> state : this.m_FSM.getAllStates()) {
        if (state.getState().equals(transition.nextState)) {
          this.callAction(
              new FSMAction<T>(
                  this.getCurrentState(), recvdMsgId, transition.action, transition.nextState));
          this.m_FSM.setCurrentState(state);
          return true;
        }
      }
    }
    return false;
  }

  /**
   * Method to call the Action handler.<br>
   *
   * @param action Action object to pass to the Action handler
   */
  public void callAction(FSMAction<T> action) {
    if (null != this.m_actionHandler) {
      m_actionHandler.accept(action);
    }
  }

  /**
   * Method returns the current state of the FiniteStateMachine<br>
   *
   * @return Current state of the FiniteStateMachine
   */
  public T getCurrentState() {
    return this.m_FSM.getCurrentState().getState();
  }
}
