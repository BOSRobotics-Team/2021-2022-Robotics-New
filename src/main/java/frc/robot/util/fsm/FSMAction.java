package frc.robot.util.fsm;

/** Action object for Finite State Machine */
public class FSMAction<T extends Enum<T>> {

  public T currentState; // the current state of the FSM
  public String messageID; // the message being processed
  public String action; // the action to take
  public T nextState; // the state to transition to

  /**
   * This constructor creates a FSMAction object with the params specified <br>
   *
   * @param currState Specifies the current state of FSM
   * @param msg Specifies the transition message
   * @param act Specifies action string
   * @param nState Specifies next
   */
  FSMAction(T currState, String msg, String act, T nState) {
    currentState = currState;
    messageID = msg;
    action = act;
    nextState = nState;
  }
}
