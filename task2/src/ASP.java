import fr.uga.pddl4j.heuristics.state.StateHeuristic;
import fr.uga.pddl4j.parser.DefaultParsedProblem;
import fr.uga.pddl4j.parser.RequireKey;
import fr.uga.pddl4j.plan.Plan;
import fr.uga.pddl4j.plan.SequentialPlan;
import fr.uga.pddl4j.planners.AbstractPlanner;
import fr.uga.pddl4j.planners.ProblemNotSupportedException;
import fr.uga.pddl4j.planners.SearchStrategy;
import fr.uga.pddl4j.planners.statespace.search.StateSpaceSearch;
import fr.uga.pddl4j.problem.DefaultProblem;
import fr.uga.pddl4j.problem.Problem;
import fr.uga.pddl4j.problem.State;
import fr.uga.pddl4j.problem.operator.Action;
import fr.uga.pddl4j.problem.operator.ConditionalEffect;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import picocli.CommandLine;
import java.lang.management.ManagementFactory;
import java.lang.management.MemoryMXBean;
import java.lang.management.MemoryUsage;
import java.util.*;


@CommandLine.Command(name = "ASP",
        version = "ASP 1.0",
        description = "Solves a specified planning problem using A* search strategy with different heuristic.",
        sortOptions = false,
        mixinStandardHelpOptions = true,
        headerHeading = "Usage:%n",
        synopsisHeading = "%n",
        descriptionHeading = "%nDescription:%n%n",
        parameterListHeading = "%nParameters:%n",
        optionListHeading = "%nOptions:%n")
public class ASP extends AbstractPlanner {

    private static final Logger LOGGER = LogManager.getLogger(ASP.class.getName());
    private double heuristicWeight;
    private StateHeuristic.Name heuristic;

    /*
     * Campi aggiuntivi:
     *     • useNewHeuristic = per poter usare l'ueristica definita ad Hoc
     */
    private int useNewHeuristic;

    /*
     * Dal problem.pddl file di cui viene effettuato il parsing,
     * viene istanziato l’oggetto della classe DefaultParsedProblem
     * relativo al problema di planning;
     */
    @Override
    public Problem instantiate(DefaultParsedProblem problem) {
        final Problem pb = new DefaultProblem(problem);
        pb.instantiate();
        return pb;
    }

    //Metodo principale della classe
    @Override
    public Plan solve(final Problem problem) throws ProblemNotSupportedException {
        //Andiamo a vedere per prima cosa se il problema è risolvibile dal plan costruito
        if (!this.isSupported(problem)) { throw new ProblemNotSupportedException("Cannot solve the problem");}

        //ASTAR
        //mediante la seguente variabile di classe andiamo a scegliere l'euristica di riferimento
        if (this.getHeuristicNew() == 1){

            LOGGER.info("* Starting A* Search with Custom Heuristic \n");

                //Per poter tenere traccia della memoria usata dall'algoritmo
                MemoryMXBean memoryBean = ManagementFactory.getMemoryMXBean();
                MemoryUsage beforeHeapMemoryUsage = memoryBean.getHeapMemoryUsage();
                long beforeUsedMemory = beforeHeapMemoryUsage.getUsed();

                //Per tenere traccia del tempo usato dall'algoritmo
                final long begin = System.currentTimeMillis();
                //Lanciamo l'algoritmo
                final Plan plan = this.customAstar(problem);
                final long end = System.currentTimeMillis();

                MemoryUsage afterHeapMemoryUsage = memoryBean.getHeapMemoryUsage();
                long afterUsedMemory = afterHeapMemoryUsage.getUsed();

                long memoryUsedByCustomAstar = afterUsedMemory - beforeUsedMemory;

                if (plan != null) {
                    LOGGER.info("* A* search succeeded\n");
                    this.getStatistics().setTimeToSearch(end - begin);
                    this.getStatistics().setMemoryUsedToSearch(memoryUsedByCustomAstar);
                } else {
                    LOGGER.info("* A* search failed\n");
                }
                return plan;
            }
            else{
                //In questo caso usiamo l'algoritmo A* con una delle euristiche fornite dal framework
                LOGGER.info("* Starting A* search with heuristic: "+ this.getHeuristic()+"\n");
                StateSpaceSearch search = StateSpaceSearch.getInstance(SearchStrategy.Name.ASTAR,
                        this.getHeuristic(), this.getHeuristicWeight(), this.getTimeout());
                LOGGER.info("* Starting A* search \n");
                // Cerchiamo una soluzione
                Plan plan = search.searchPlan(problem);
                
                if (plan != null) {
                    LOGGER.info("* A* search succeeded\n");
                    this.getStatistics().setTimeToSearch(search.getSearchingTime());
                    this.getStatistics().setMemoryUsedToSearch(search.getMemoryUsed());
                } else {
                    LOGGER.info("* A* search failed\n");
                }

                return plan;
            }
    }

    /*
     * Metodo di verifica che il problema rispetti i requisiti
     * che ne consentono la risoluzione
     */
    @Override
    public boolean isSupported(Problem problem) {
        return !problem.getRequirements().contains(RequireKey.ACTION_COSTS)
                && !problem.getRequirements().contains(RequireKey.CONSTRAINTS)
                && !problem.getRequirements().contains(RequireKey.CONTINOUS_EFFECTS)
                && !problem.getRequirements().contains(RequireKey.DERIVED_PREDICATES)
                && !problem.getRequirements().contains(RequireKey.DURATIVE_ACTIONS)
                && !problem.getRequirements().contains(RequireKey.DURATION_INEQUALITIES)
                && !problem.getRequirements().contains(RequireKey.FLUENTS)
                && !problem.getRequirements().contains(RequireKey.GOAL_UTILITIES)
                && !problem.getRequirements().contains(RequireKey.METHOD_CONSTRAINTS)
                && !problem.getRequirements().contains(RequireKey.NUMERIC_FLUENTS)
                && !problem.getRequirements().contains(RequireKey.OBJECT_FLUENTS)
                && !problem.getRequirements().contains(RequireKey.PREFERENCES)
                && !problem.getRequirements().contains(RequireKey.TIMED_INITIAL_LITERALS)
                && !problem.getRequirements().contains(RequireKey.HIERARCHY);
    }

    //Metodo per settare i pesi dell'euristica
    @CommandLine.Option(names = {"-w", "--weight"}, defaultValue = "1.0",
            paramLabel = "<weight>", description = "Set the weight of the heuristic (preset 1.0).")
    public void setHeuristicWeight(final double weight) {
        if (weight <= 0) {
            throw new IllegalArgumentException("Weight <= 0");
        }
        this.heuristicWeight = weight;
    }

    //Metodo per settare la nuova euristica
    // 0 -> Si usa l'algorimto di ricerca con un eristica di base
    // 1 -> Si usa l'algoritmo di ricerca con l'euristica definita ad hoc
    @CommandLine.Option(names = {"-en", "--heuristicNew"}, defaultValue = "0",
            description = "Set the heuristic : 1")
    public void setHeuristicNew(int heuristicsNew) {
        this.useNewHeuristic = heuristicsNew;
    }

    public final int getHeuristicNew() {
        return this.useNewHeuristic;
    }

    //Metodo per settare l'euristica
    @CommandLine.Option(names = {"-e", "--heuristic"}, defaultValue = "FAST_FORWARD",
            description = "Set the heuristic : AJUSTED_SUM, AJUSTED_SUM2, AJUSTED_SUM2M, COMBO, "
                    + "MAX, FAST_FORWARD SET_LEVEL, SUM, SUM_MUTEX (preset: FAST_FORWARD)")
    public void setHeuristic(StateHeuristic.Name heuristic) {
        this.heuristic = heuristic;
    }

    public final StateHeuristic.Name getHeuristic() {
        return this.heuristic;
    }

    public final double getHeuristicWeight() {
        return this.heuristicWeight;
    }

    //Algoritmo A* modificato ad hoc (definito come nella guida di PDDL4J ma con l'euristica custom)
    public Plan customAstar(Problem problem){

        //Costruiamo la nostra euristica
        CustomHeuristic heuristic = new CustomHeuristic(problem);

        // Ci prendiamo lo stato iniziale
        final State init = new State(problem.getInitialState());

        // Inizializziamo l'insieme dei nodi visitati
        final Set<Node> close = new HashSet<>();


        final double weight = this.getHeuristicWeight();
        final PriorityQueue<Node> open = new PriorityQueue<>(100, new Comparator<Node>() {
            public int compare(Node n1, Node n2) {
                double f1 = weight * n1.getHeuristic() + n1.getCost();
                double f2 = weight * n2.getHeuristic() + n2.getCost();
                return Double.compare(f1, f2);
            }
        });

        // Definiamo la radice dell'albero di ricerca
        final Node root = new Node(init, null, -1, 0, heuristic.estimate(init, problem.getGoal()));


        open.add(root);
        Plan plan = null;

        // Settiamo il timeout per la ricerca
        final int timeout = this.getTimeout() * 1000;
        long time = 0;

        // Inizio ricerca
        while (!open.isEmpty() && plan == null && time < timeout) {

            // Aggiungiamo il nodo a quelli visitati
            final Node current = open.poll();
            close.add(current);

            //Se il goal è soddisfatto allora restituiamo il plan
            if (current.satisfy(problem.getGoal())) {
                return this.extractPlan(current, problem);
            } else {
                //Altrimenti proviamo ad applicare le azioni al nodo
                for (int i = 0; i < problem.getActions().size(); i++) {
                    // Ci prendiamo le azioni
                    Action a = problem.getActions().get(i);
                    // Verifichiamo se l'azione è applicabile al problema
                    if (a.isApplicable(current)) {
                        Node next = new Node(current);
                        // Applichiamo l'effetto dell'azione
                        final List<ConditionalEffect> effects = a.getConditionalEffects();
                        for (ConditionalEffect ce : effects) {
                            if (current.satisfy(ce.getCondition())) {
                                next.apply(ce.getEffect());
                            }
                        }
                        //Andiamo ad inserire le informazioni al nuovo nodo
                        final double g = current.getCost() + 1;
                        if (!close.contains(next)) {
                            next.setCost(g);
                            next.setParent(current);
                            next.setAction(i);
                            next.setHeuristic(heuristic.customEstimate(next, problem.getGoal(), a, current.getHeuristic()));
                            open.add(next);
                        }
                    }
                }
            }
        }
        //Restituiamo il plan -> Null se vuoto
        return plan;
    }

    //Metodo definito per poter estrarre il plan, invocato nell'algoritmo customAstar
    private Plan extractPlan(final Node node, final Problem problem) {
        Node n = node;
        final Plan plan = new SequentialPlan();
        while (n.getAction() != -1) {
            final Action a = problem.getActions().get(n.getAction());
            plan.add(0, a);
            n = n.getParent();
        }
        return plan;
    }

    public static void main(String[] args) {
        try {
            final ASP planner = new ASP();
            CommandLine cmd = new CommandLine(planner);
            cmd.execute(args);
        } catch (IllegalArgumentException e) {
            LOGGER.fatal(e.getMessage());
        }

    }
}
