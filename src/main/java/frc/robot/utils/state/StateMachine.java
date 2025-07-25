// package frc.robot.utils.state;

// import frc.robot.utils.telemetry.TelemetryManager;
// import java.util.HashMap;
// import java.util.Map;
// import java.util.function.BooleanSupplier;

// public class StateMachine<T extends Enum<T>> {
//     private final Map<T, State<T>> states = new HashMap<>();
//     private T currentState;
//     private T previousState;
//     private final String name;
//     private final TelemetryManager telemetry = TelemetryManager.getInstance();
    
//     public StateMachine(String name, T initialState) {
//         this.name = name;
//         this.currentState = initialState;
//         this.previousState = initialState;
//     }
    
//     public void addState(T stateEnum, State<T> state) {
//         states.put(stateEnum, state);
//     }
    
//     public void addTransition(T from, T to, BooleanSupplier condition) {
//         State<T> state = states.get(from);
//         if (state != null) {
//             state.addTransition(to, condition);
//         }
//     }
    
//     public void update() {
//         State<T> state = states.get(currentState);
//         if (state != null) {
//             state.execute();
            
//             // Check for transitions
//             T nextState = state.getNextState();
//             if (nextState != null && nextState != currentState) {
//                 transitionTo(nextState);
//             }
//         }
        
//         // Publish telemetry
        
//     }
    
//     private void transitionTo(T newState) {
//         State<T> currentStateObj = states.get(currentState);
//         if (currentStateObj != null) {
//             currentStateObj.onExit();
//         }
        
//         previousState = currentState;
//         currentState = newState;
        
//         State<T> newStateObj = states.get(newState);
//         if (newStateObj != null) {
//             newStateObj.onEnter();
//         }
        
       
//     }
    
//     public T getCurrentState() {
//         return currentState;
//     }
    
//     public T getPreviousState() {
//         return previousState;
//     }
    
//     public void forceState(T state) {
//         transitionTo(state);
//     }
    
//     public static abstract class State<T extends Enum<T>> {
//         private final Map<T, BooleanSupplier> transitions = new HashMap<>();
        
//         public void addTransition(T to, BooleanSupplier condition) {
//             transitions.put(to, condition);
//         }
        
//         public T getNextState() {
//             for (Map.Entry<T, BooleanSupplier> entry : transitions.entrySet()) {
//                 if (entry.getValue().getAsBoolean()) {
//                     return entry.getKey();
//                 }
//             }
//             return null;
//         }
        
//         public abstract void onEnter();
//         public abstract void execute();
//         public abstract void onExit();
//     }
// }
