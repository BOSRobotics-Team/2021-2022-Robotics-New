package frc.robot.util.fsm;

/** Transition object for Finite State Machine */
public class FSMTransition<T extends Enum<T>> {

  public String action; // The specified action string
  public T nextState; // The specified next state

  /**
   * This constructor creates a FSMTransition object with the params specified <br>
   *
   * @param nState Specifies the next state
   * @param actn Specifies action string
   */
  FSMTransition(T nState, String actn) {
    nextState = nState;
    action = actn;
  }

  /**
   * This constructor creates a FSMTransition object with the params specified <br>
   *
   * @param nState Specifies the next state
   */
  FSMTransition(T nState) {
    this(nState, "");
  }
}
