import fr.uga.pddl4j.heuristics.state.RelaxedGraphHeuristic;
import fr.uga.pddl4j.planners.statespace.search.Node;
import fr.uga.pddl4j.problem.Problem;
import fr.uga.pddl4j.problem.State;
import fr.uga.pddl4j.problem.operator.Action;
import fr.uga.pddl4j.problem.operator.Condition;
import fr.uga.pddl4j.problem.Fluent;
import fr.uga.pddl4j.util.BitVector;

public final class CustomHeuristic extends RelaxedGraphHeuristic {

    private final int totalBoxes;
    private final int totalWorkstations;
    private Problem problem;

    public CustomHeuristic(Problem problem) {
        super(problem);
        super.setAdmissible(false); // Non ammissibile per favorire performance

        // Conta il numero totale di scatole e workstation basandosi sui fluents
        this.totalBoxes = countFluentsByType(problem, "box");
        this.totalWorkstations = countFluentsByType(problem, "workstation");
        this.problem = problem;
    }

    @Override
    public int estimate(State state, Condition goal) {
        super.setGoal(goal);
        super.expandRelaxedPlanningGraph(state);

        // Ottieni la capacità del carrier dinamicamente
        int carrierCapacity = getCarrierCapacity(state);

        // Stima i viaggi necessari
        int tripsNeeded = (int) Math.ceil((double) totalBoxes / carrierCapacity);

        // Stima la distanza totale tra il magazzino e le workstation
        int distanceEstimate = estimateTotalDistance();

        // Combina le stime
        return tripsNeeded * distanceEstimate;
    }

    @Override
    public double estimate(Node node, Condition goal) {
        return this.estimate((State) node, goal);
    }

    public double customEstimate(State state, Condition goal, Action action, double currentHeuristic) {
        // Penalità per azioni specifiche
        double actionPenalty = action.getName().contains("move") ? 2.0 : 0.0;
        actionPenalty += action.getName().contains("load") ? 1.0 : 0.0;
        actionPenalty += action.getName().contains("unload") ? 0.5 : 0.0;

        // Combina stima dei viaggi, distanze e penalità
        return estimate(state, goal) + actionPenalty + (0.1 * currentHeuristic);
    }

    private int getCarrierCapacity(State state) {
        int capacity = 0;

        // Scansiona i fluents definiti nel problema per identificare gli slot
        for (Fluent fluent : this.problem.getFluents()) {
            String fluentName = fluent.toString();

            // Verifica se il fluent è di tipo "slot"
            if (fluentName.contains("slot") && fluentName.startsWith("(available")) {
                int fluentIndex = this.problem.getFluents().indexOf(fluent);

                if (fluentIndex != -1) {
                    // Crea un BitVector per rappresentare il fluent
                    BitVector positiveFluents = new BitVector();
                    positiveFluents.set(fluentIndex);

                    // Costruisce la condizione
                    Condition slotCondition = new Condition(positiveFluents, new BitVector());

                    // Verifica se lo slot è disponibile nello stato attuale
                    if (state.satisfy(slotCondition)) {
                        capacity++;
                    }
                }
            }
        }

        return Math.max(capacity, 1); // Garantisce che la capacità sia almeno 1
    }

    private int estimateTotalDistance() {
        // Stima la distanza totale tra il magazzino e tutte le workstation
        return totalWorkstations; // Esempio: Supponiamo che ogni workstation richieda un costo unitario
    }

    private int countFluentsByType(Problem problem, String type) {
        int count = 0;

        // Scorri i fluents definiti nel problema
        for (Fluent fluent : problem.getFluents()) {
            String fluentName = fluent.toString();

            // Verifica se il nome del fluent corrisponde al tipo richiesto
            if (fluentName.contains(type)) {
                count++;
            }
        }

        return count;
    }
}
