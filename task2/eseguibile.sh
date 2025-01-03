DOMAIN1=pddl/21/"domain.pddl"
PROBLEM1=pddl/21/"problem.pddl"
DOMAIN2=pddl/22/"domain2.pddl"
PROBLEM2=pddl/22/"problem2.pddl"


if [[ ! -f "$DOMAIN1" ]]; then
    echo "Il file di dominio non esiste: $DOMAIN1"
    exit 1
fi

if [[ ! -f "$PROBLEM1" ]]; then
    echo "Il file del problema non esiste: $PROBLEM1"
    exit 1
fi

if [[ ! -f "$DOMAIN2" ]]; then
    echo "Il file di dominio non esiste: $DOMAIN2"
    exit 1
fi

if [[ ! -f "$PROBLEM2" ]]; then
    echo "Il file del problema non esiste: $PROBLEM2"z
    exit 1
fi


echo "Compilando le classi Java ... "
javac -d classes -cp lib/pddl4j-4.0.0.jar src/*.java

echo "Compilazione avvenuta con successo!"
echo "Eseguo Istanza 1"

echo "FastForward"
java -cp classes:lib/pddl4j-4.0.0.jar fr.uga.pddl4j.planners.statespace.FF $DOMAIN1 $PROBLEM1 > results/21/ff_result.txt

echo "A* con euristica FastForward"
java -cp classes:lib/pddl4j-4.0.0.jar ASP $DOMAIN1 $PROBLEM1 > results/21/a_ff_result.txt

echo "A* con euristica AJUSTED_SUM"
java -cp classes:lib/pddl4j-4.0.0.jar ASP $DOMAIN1 $PROBLEM1 -e AJUSTED_SUM > results/21/a_as_result.txt

echo "A* con euristica personalizzata"
java -cp classes:lib/pddl4j-4.0.0.jar ASP $DOMAIN1 $PROBLEM1 -en 1 > results/21/a_custom_result.txt



echo "Eseguo Istanza 2"

echo "FastForward"
java -cp classes:lib/pddl4j-4.0.0.jar fr.uga.pddl4j.planners.statespace.FF $DOMAIN2 $PROBLEM2 > results/22/ff_result.txt

echo "A* con euristica FastForward"
java -cp classes:lib/pddl4j-4.0.0.jar ASP $DOMAIN2 $PROBLEM2 > results/22/a_ff_result.txt

echo "A* con euristica AJUSTED_SUM"
java -cp classes:lib/pddl4j-4.0.0.jar ASP $DOMAIN2 $PROBLEM2 -e AJUSTED_SUM > results/22/a_as_result.txt

echo "A* con euristica personalizzata"
java -cp classes:lib/pddl4j-4.0.0.jar ASP $DOMAIN2 $PROBLEM2 -en 1 > results/22/a_custom_result.txt


