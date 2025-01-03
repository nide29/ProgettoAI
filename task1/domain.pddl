(define (domain manufacturing)
    
    (:requirements :strips :typing :equality :adl :universal-preconditions)
    

    ;; Tipologie degli oggetti gestiti
    (:types 
        location box content robot workstation
    )


    ;; Predicati 
    (:predicates 
        (located_at ?entity - (either robot workstation box content) ?loc - location)
        (is_empty ?box - box)
        (contains ?box - box ?item - content)
        (inside_ws ?entity - (either robot box content) ?ws - workstation)
        (holding ?robot - robot ?box - box)
        (available ?robot - robot) ; Il robot è libero se non sta trasportando nulla
        (linked ?loc1 ?loc2 - location)
        (is_warehouse ?loc - location) ; Identifica il magazzino
    )


    ;; Azioni

    ; Spostare il robot tra due location adiacenti
    (:action move_robot
        :parameters (?r - robot ?from ?to - location)
        :precondition (and (located_at ?r ?from) (linked ?from ?to))
        :effect (and (not (located_at ?r ?from)) (located_at ?r ?to))
    )

    ; Il robot entra in una workstation
    (:action enter_workstation
        :parameters (?r - robot ?loc - location ?ws - workstation)
        :precondition (and (located_at ?ws ?loc) (located_at ?r ?loc) (not (inside_ws ?r ?ws)))
        :effect (and (not (located_at ?r ?loc)) (inside_ws ?r ?ws))
    )

    ; Il robot esce dalla workstation
    (:action exit_workstation
        :parameters (?r - robot ?loc - location ?ws - workstation)
        :precondition (and (located_at ?ws ?loc) (inside_ws ?r ?ws))
        :effect (and (not (inside_ws ?r ?ws)) (located_at ?r ?loc))
    )

    ; Il robot lascia una scatola nella workstation
    (:action drop_box_in_ws
        :parameters (?r - robot ?box - box ?ws - workstation)
        :precondition (and (inside_ws ?r ?ws) (holding ?r ?box))
        :effect (and (not (holding ?r ?box)) (available ?r) (inside_ws ?box ?ws))
    )

    ; Il robot lascia una scatola in una location
    (:action drop_box_in_loc
        :parameters (?r - robot ?box - box ?loc - location)
        :precondition (and (located_at ?r ?loc) (holding ?r ?box))
        :effect (and (not (holding ?r ?box)) (available ?r) (located_at ?box ?loc))
    )

    ; Il robot raccoglie una scatola da una workstation
    (:action pick_box_from_ws
        :parameters (?r - robot ?box - box ?ws - workstation)
        :precondition (and (inside_ws ?box ?ws) (inside_ws ?r ?ws) (available ?r))
        :effect (and (holding ?r ?box) (not (available ?r)) (not (inside_ws ?box ?ws)))
    )

    ; Il robot raccoglie una scatola da una location
    (:action pick_box_from_loc
        :parameters (?r - robot ?box - box ?loc - location)
        :precondition (and (located_at ?box ?loc) (located_at ?r ?loc) (available ?r))
        :effect (and (holding ?r ?box) (not (available ?r)) (not (located_at ?box ?loc)))
    )

    ; Il robot svuota una scatola in una workstation
    (:action unload_box_in_ws
        :parameters (?r - robot ?box - box ?ws - workstation ?content - content)
        :precondition (and (available ?r) (inside_ws ?r ?ws) (inside_ws ?box ?ws) (contains ?box ?content))
        :effect (and (not (contains ?box ?content)) (is_empty ?box) (inside_ws ?content ?ws))
    )

    ; Il robot riempie una scatola in una workstation
    (:action load_box_in_ws
        :parameters (?r - robot ?box - box ?ws - workstation ?content - content)
        :precondition (and (available ?r) (inside_ws ?r ?ws) (inside_ws ?box ?ws) (inside_ws ?content ?ws) (is_empty ?box))
        :effect (and (not (inside_ws ?content ?ws)) (contains ?box ?content) (not (is_empty ?box)))
    )

    ; Il robot svuota una scatola in una location
    (:action unload_box_in_loc
        :parameters (?r - robot ?box - box ?content - content ?loc - location)
        :precondition (and (available ?r) (located_at ?r ?loc) (located_at ?box ?loc) (contains ?box ?content))
        :effect (and (not (contains ?box ?content)) (is_empty ?box) (located_at ?content ?loc))
    )

    ; Il robot riempie una scatola in una location
    (:action load_box_in_loc
        :parameters (?r - robot ?box - box ?content - content ?loc - location)
        :precondition (and (not (is_warehouse ?loc)) (available ?r) (located_at ?r ?loc) (located_at ?box ?loc) (located_at ?content ?loc) (is_empty ?box))
        :effect (and (not (located_at ?content ?loc)) (contains ?box ?content) (not (is_empty ?box)))
    )

    ; Il robot riempie una scatola nel magazzino
    (:action load_box_in_warehouse
        :parameters (?r - robot ?box - box ?content - content ?loc - location)
        :precondition (and (is_warehouse ?loc) (available ?r) (located_at ?r ?loc) (located_at ?box ?loc) (located_at ?content ?loc) (is_empty ?box))
        :effect (and (contains ?box ?content) (not (is_empty ?box)))
    )
)
