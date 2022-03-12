package frc.robot.util.fsm;

import java.util.HashMap;

/** State object for Finite State Machine */
public class FSMState<T extends Enum<T>> {

  private T m_state; // State id for this object
  private HashMap<String, FSMTransition<T>> m_transitionMap; // transition map

  /*
   * Tags to be read from XML
   */
  public static final String kStateTag = "STATE";
  public static final String kIdTag = "id";
  public static final String kNextStateTag = "nextState";
  public static final String kActionTag = "action";

  /**
   * This Constructor allows to create a FiniteStateMachine with the initial state <br>
   *
   * @param state Specifies the state of this FSMState
   */
  public FSMState(T state) {
    this.m_state = state;
  }

  /**
   * This Constructor creates a FSMState with initial state and sets the transitions as per the
   * transition map.<br>
   *
   * @param state Initial state of the FSMState
   * @param map Transition Map of states.
   */
  public FSMState(T state, HashMap<String, FSMTransition<T>> map) {
    this.m_state = state;
    this.m_transitionMap = map;
  }

  /**
   * Method to add additional Messages along with their own transition Action
   *
   * @param message Specified message to add
   * @param action Specified action for the given message
   * @param nextState Specified next state to transition to
   */
  public void addMessage(String message, String action, T nextState) {
    this.m_transitionMap.put(message, new FSMTransition<T>(nextState, action));
  }

  /**
   * Method to allow addition of Messages along with their own corresponding Action
   *
   * @param message Specified message to add
   * @param action Specified transition action for the given message
   */
  public void addMessage(String message, FSMTransition<T> action) {
    this.m_transitionMap.put(message, action);
  }

  /**
   * Method to set the entire Transition Map.<br>
   *
   * @param transitionMap Transition Map for this State
   */
  public void addMessages(HashMap<String, FSMTransition<T>> transitionMap) {
    this.m_transitionMap = transitionMap;
  }

  /**
   * Method to return the entire Transition Map<br>
   *
   * @return Transition Map for this state
   */
  public HashMap<String, FSMTransition<T>> getTransitionMap() {
    return this.m_transitionMap;
  }

  /**
   * Method to return the state of this FSMState object
   *
   * @return state ID
   */
  public T getState() {
    return this.m_state;
  }
}
