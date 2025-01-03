(define (problem istanza1)
    (:domain manufacturing)

    ;; Oggetti nel problema
    (:objects
        warehouse location1 location2 - location
        robot1 - robot
        box1 box2 box3 - box
        bolt valve - content
        ws1 ws2 ws3 - workstation
    )

    ;; Stato iniziale
    (:init
        ; Posizionamento iniziale
        (located_at robot1 warehouse)

        ; Il magazzino è definito come tale
        (is_warehouse warehouse)

        (located_at box1 warehouse)
        (located_at box2 warehouse)
        (located_at box3 warehouse)

        (located_at bolt warehouse)
        (located_at valve warehouse)

        
        ; I box iniziano vuoti
        (is_empty box1)
        (is_empty box2)
        (is_empty box3)

        ; Posizionamento delle workstation
        (located_at ws1 location1)
        (located_at ws2 location2)
        (located_at ws3 location2)

        ; Il robot è disponibile
        (available robot1)

        ; Collegamenti tra location
        (linked warehouse location1)
        (linked location1 location2)
        (linked location1 warehouse)
        (linked location2 location1)

        
    )

    ;; Obiettivi
    (:goal
        (and
            ;; ws2 richiede un bolt
            (inside_ws bolt ws2)

            ;; ws3 richiede un bolt e valve
            (inside_ws bolt ws3)
            (inside_ws valve ws3)

            ;; ws1 non richiede nulla
            (inside_ws bolt ws1)
            (inside_ws valve ws1)
        )
    )
)
