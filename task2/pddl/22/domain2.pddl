(define (domain manufacturing)
    
    (:requirements :strips :typing :equality :adl :universal-preconditions)
    

    ;; Tipologie degli oggetti gestiti
    (:types 
        location box content robot workstation carrier slot
    )


    ;; Predicati 
    (:predicates 
        (located_at ?entity - (either robot workstation box content carrier) ?loc - location)
        (is_empty ?box - box)
        (contains ?box - box ?item - content)
        (inside_ws ?entity - (either robot box content) ?ws - workstation)
        (carrying ?carrier - carrier ?box - box)   ; Si utiilzza Carrier invece del Robot 
        ;(available ?robot - robot) ; Questo predicato viene commentato in quanto utilizzando il carrier non c'è bisogno di preoccuparsi dello stato (libero o meno) del robot
        (linked ?loc1 ?loc2 - location)
        (is_warehouse ?loc - location) ; Identifica il magazzino

        ; NUOVI PREDICATI introdotti per soddisfare le richieste
        (joined ?robot - robot ?carrier - carrier)
        (handle ?carrier - carrier ?slot - slot) ; Il carrier possiede questo slot
        (available ?slot - slot) ; Uno slot è libero per essere popolato
    )


    ;; Azioni

    ; Spostare il robot tra due location adiacenti (che non sia la warehouse)
    (:action move_robot
        :parameters (?r - robot ?from ?to - location ?c - carrier)
        :precondition (and (located_at ?r ?from) (linked ?from ?to) (not (is_warehouse ?to)) (joined ?r ?c))
        :effect (and (not (located_at ?r ?from)) (located_at ?r ?to))
    )

    ; Spostare il robot tra due location adiacenti (che sia la warehouse)
    (:action move_robot_warehouse
        :parameters (?r - robot ?from ?to - location ?c - carrier)
        :precondition (and (located_at ?r ?from) (linked ?from ?to) (is_warehouse ?to) (joined ?r ?c)
            (forall (?slot - slot)
                (and (handle ?c ?slot) (available ?slot))
            )
        )
        :effect (and (not (located_at ?r ?from)) (located_at ?r ?to))
        ; Vi è la necessità di differenziare queste azioni in quanto nella warehouse si può accedere solo con il carrier vuoto
    )
    
    ; Il robot entra in una workstation
    (:action enter_workstation
        :parameters (?r - robot ?loc - location ?ws - workstation ?c - carrier)
        :precondition (and (located_at ?ws ?loc) (located_at ?r ?loc) (not (inside_ws ?r ?ws)) (joined ?r ?c))
        :effect (and (not (located_at ?r ?loc)) (inside_ws ?r ?ws))
    )

    ; Il robot esce dalla workstation
    (:action exit_workstation
        :parameters (?r - robot ?loc - location ?ws - workstation ?c - carrier)
        :precondition (and (located_at ?ws ?loc) (not(located_at ?r ?loc)) (inside_ws ?r ?ws) (joined ?r ?c))
        :effect (and (not (inside_ws ?r ?ws)) (located_at ?r ?loc))
    )

    ; Il robot lascia una scatola (trasportata nel carrier) nella workstation
    (:action drop_box_in_ws
        :parameters (?r - robot ?box - box ?ws - workstation ?c - carrier ?slot - slot)
        :precondition (and (inside_ws ?r ?ws) (carrying ?c ?box) (joined ?r ?c) (handle ?c ?slot))
        :effect (and (not (carrying ?c ?box)) (available ?slot) (inside_ws ?box ?ws))
    )

    ; Il robot lascia una scatola (trasportata nel carrier) in una location
    (:action drop_box_in_loc
        :parameters (?r - robot ?box - box ?loc - location ?c - carrier ?slot - slot)
        :precondition (and (located_at ?r ?loc) (carrying ?c ?box) (joined ?r ?c) (handle ?c ?slot))
        :effect (and (not (carrying ?c ?box)) (available ?slot) (located_at ?box ?loc))
    )

    ; Il robot raccoglie una scatola da una workstation
    (:action pick_box_from_ws
        :parameters (?r - robot ?box - box ?ws - workstation ?c - carrier ?slot - slot)
        :precondition (and (inside_ws ?box ?ws) (inside_ws ?r ?ws) (joined ?r ?c) (handle ?c ?slot) (available ?slot))
        :effect (and (carrying ?c ?box) (not (available ?slot)) (not (inside_ws ?box ?ws)))
    )

    ; Il robot raccoglie una scatola da una location
    (:action pick_box_from_loc
        :parameters (?r - robot ?box - box ?loc - location ?c - carrier ?slot - slot) 
        :precondition (and (located_at ?box ?loc) (located_at ?r ?loc) (joined ?r ?c) (handle ?c ?slot) (available ?slot))
        :effect (and (carrying ?c ?box) (not (available ?slot)) (not (located_at ?box ?loc)))
    )

    ; Il robot riempie una scatola in una workstation
    (:action load_box_in_ws
        :parameters (?r - robot ?box - box ?ws - workstation ?content - content ?c - carrier)
        :precondition (and  (inside_ws ?r ?ws) (inside_ws ?box ?ws) (inside_ws ?content ?ws) (is_empty ?box) (joined ?r ?c))
        :effect (and (not (inside_ws ?content ?ws)) (contains ?box ?content) (not (is_empty ?box)))
    )

    ; Il robot svuota una scatola in una workstation
    (:action unload_box_in_ws
        :parameters (?r - robot ?box - box ?ws - workstation ?content - content ?c - carrier)
        :precondition (and (inside_ws ?r ?ws) (inside_ws ?box ?ws) (contains ?box ?content) (joined ?r ?c))
        :effect (and (not (contains ?box ?content)) (is_empty ?box) (inside_ws ?content ?ws))
    )

    ; Il robot riempie una scatola in una location
    (:action load_box_in_loc
        :parameters (?r - robot ?box - box ?content - content ?loc - location ?c - carrier)
        :precondition (and (not (is_warehouse ?loc)) (located_at ?r ?loc) (located_at ?box ?loc) (located_at ?content ?loc) (is_empty ?box) (joined ?r ?c))
        :effect (and (not (located_at ?content ?loc)) (contains ?box ?content) (not (is_empty ?box)))
    )

    ; Il robot svuota una scatola in una location
    (:action unload_box_in_loc
        :parameters (?r - robot ?box - box ?content - content ?loc - location ?c - carrier)
        :precondition (and (located_at ?r ?loc) (located_at ?box ?loc) (contains ?box ?content) (joined ?r ?c))
        :effect (and (not (contains ?box ?content)) (is_empty ?box) (located_at ?content ?loc))
    )

    ; Il robot riempie una scatola nel magazzino
    (:action load_box_in_warehouse
        :parameters (?r - robot ?box - box ?content - content ?loc - location ?c - carrier)
        :precondition (and (is_warehouse ?loc) (located_at ?r ?loc) (located_at ?box ?loc) (located_at ?content ?loc) (is_empty ?box) (joined ?r ?c))
        :effect (and (contains ?box ?content) (not (is_empty ?box)))
    )
)
