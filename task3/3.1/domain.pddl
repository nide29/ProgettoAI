(define (domain manufacturing_temporal)
    
    (:requirements :strips :typing :equality :adl :durative-actions :universal-preconditions)
    

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
        (carrying ?carrier - carrier ?box - box)
        (linked ?loc1 ?loc2 - location)
        (is_warehouse ?loc - location)
        (joined ?robot - robot ?carrier - carrier)
        (handle ?carrier - carrier ?slot - slot)
        (available ?slot - slot)
    )


    ;; Durative Actions

    ; Spostare il robot tra due location adiacenti
    (:durative-action move_robot
        :parameters (?r - robot ?from ?to - location ?c - carrier)
        :duration (= ?duration 3)
        :condition (and (at start (located_at ?r ?from))
                        (at start (linked ?from ?to))
                        (over all (joined ?r ?c)))
        :effect (and (at start (not (located_at ?r ?from)))
                     (at end (located_at ?r ?to)))
    )

    ; Spostare il robot verso il magazzino
    (:durative-action move_robot_warehouse
        :parameters (?r - robot ?from ?to - location ?c - carrier)
        :duration (= ?duration 3)
        :condition (and (at start (located_at ?r ?from))
                        (at start (linked ?from ?to))
                        (at start (is_warehouse ?to))
                        (over all (joined ?r ?c))
                        (at start (forall (?slot - slot) (and (handle ?c ?slot) (available ?slot)))))
        :effect (and (at start (not (located_at ?r ?from)))
                     (at end (located_at ?r ?to)))
    )

    ; Il robot entra in una workstation
    (:durative-action enter_workstation
        :parameters (?r - robot ?loc - location ?ws - workstation ?c - carrier)
        :duration (= ?duration 0)
        :condition (and (at start (located_at ?ws ?loc))
                        (at start (located_at ?r ?loc))
                        (over all (joined ?r ?c)))
        :effect (and (at start (not (located_at ?r ?loc)))
                     (at end (inside_ws ?r ?ws)))
    )

    ; Il robot esce dalla workstation
    (:durative-action exit_workstation
        :parameters (?r - robot ?loc - location ?ws - workstation ?c - carrier)
        :duration (= ?duration 0)
        :condition (and (at start (inside_ws ?r ?ws))
                        (over all (joined ?r ?c)))
        :effect (and (at start (not (inside_ws ?r ?ws)))
                     (at end (located_at ?r ?loc)))
    )

    ; Il robot lascia una scatola in una workstation
    (:durative-action drop_box_in_ws
        :parameters (?r - robot ?box - box ?ws - workstation ?c - carrier ?slot - slot)
        :duration (= ?duration 2)
        :condition (and (at start (inside_ws ?r ?ws))
                        (at start (carrying ?c ?box))
                        (at start (joined ?r ?c))
                        (at start (handle ?c ?slot)))
        :effect (and (at end (not (carrying ?c ?box)))
                     (at end (available ?slot))
                     (at end (inside_ws ?box ?ws)))
    )

    ; Il robot lascia una scatola in una location
    (:durative-action drop_box_in_loc
        :parameters (?r - robot ?box - box ?loc - location ?c - carrier ?slot - slot)
        :duration (= ?duration 2)
        :condition (and (at start (located_at ?r ?loc))
                        (at start (carrying ?c ?box))
                        (at start (joined ?r ?c))
                        (at start (handle ?c ?slot)))
        :effect (and (at end (not (carrying ?c ?box)))
                     (at end (available ?slot))
                     (at end (located_at ?box ?loc)))
    )

    ; Il robot raccoglie una scatola da una workstation
    (:durative-action pick_box_from_ws
        :parameters (?r - robot ?box - box ?ws - workstation ?c - carrier ?slot - slot)
        :duration (= ?duration 2)
        :condition (and (at start (inside_ws ?box ?ws))
                        (at start (inside_ws ?r ?ws))
                        (at start (joined ?r ?c))
                        (at start (handle ?c ?slot))
                        (at start (available ?slot)))
        :effect (and (at end (carrying ?c ?box))
                     (at end (not (available ?slot)))
                     (at end (not (inside_ws ?box ?ws))))
    )

    ; Il robot raccoglie una scatola da una location
    (:durative-action pick_box_from_loc
        :parameters (?r - robot ?box - box ?loc - location ?c - carrier ?slot - slot) 
        :duration (= ?duration 2)
        :condition (and (at start (located_at ?box ?loc))
                        (at start (located_at ?r ?loc))
                        (at start (joined ?r ?c))
                        (at start (handle ?c ?slot))
                        (at start (available ?slot)))
        :effect (and (at end (carrying ?c ?box))
                     (at end (not (available ?slot)))
                     (at end (not (located_at ?box ?loc))))
    )

    ; Il robot svuota una scatola in una workstation
    (:durative-action unload_box_in_ws
        :parameters (?r - robot ?box - box ?ws - workstation ?content - content ?c - carrier)
        :duration (= ?duration 3.5)
        :condition (and (at start (inside_ws ?r ?ws))
                        (at start (inside_ws ?box ?ws))
                        (at start (contains ?box ?content))
                        (over all (joined ?r ?c)))
        :effect (and (at end (not (contains ?box ?content)))
                     (at end (is_empty ?box))
                     (at end (inside_ws ?content ?ws)))
    )

    ; Il robot riempie una scatola in una workstation
    (:durative-action load_box_in_ws
        :parameters (?r - robot ?box - box ?ws - workstation ?content - content ?c - carrier)
        :duration (= ?duration 3.5)
        :condition (and (at start (inside_ws ?r ?ws))
                        (at start (inside_ws ?box ?ws))
                        (at start (inside_ws ?content ?ws))
                        (at start (is_empty ?box))
                        (over all (joined ?r ?c)))
        :effect (and (at end (not (inside_ws ?content ?ws)))
                     (at end (contains ?box ?content))
                     (at end (not (is_empty ?box))))
    )

    ; Il robot riempie una scatola in una location
    (:durative-action load_box_in_loc
        :parameters (?r - robot ?box - box ?content - content ?loc - location ?c - carrier)
        :duration (= ?duration 3.5)
        :condition (and (at start (located_at ?r ?loc))
                        (at start (located_at ?box ?loc))
                        (at start (located_at ?content ?loc))
                        (at start (is_empty ?box))
                        (over all (joined ?r ?c)))
        :effect (and (at end (not (located_at ?content ?loc)))
                     (at end (contains ?box ?content))
                     (at end (not (is_empty ?box))))
    )

    ; Il robot svuota una scatola in una location
    (:durative-action unload_box_in_loc
        :parameters (?r - robot ?box - box ?content - content ?loc - location ?c - carrier)
        :duration (= ?duration 3.5)
        :condition (and (at start (located_at ?r ?loc))
                        (at start (located_at ?box ?loc))
                        (at start (contains ?box ?content))
                        (over all (joined ?r ?c)))
        :effect (and (at end (not (contains ?box ?content)))
                     (at end (is_empty ?box))
                     (at end (located_at ?content ?loc)))
    )

    ; Il robot riempie una scatola nel magazzino
    (:durative-action load_box_in_warehouse
        :parameters (?r - robot ?box - box ?content - content ?loc - location ?c - carrier)
        :duration (= ?duration 3.5)
        :condition (and (at start (is_warehouse ?loc))
                        (at start (located_at ?r ?loc))
                        (at start (located_at ?box ?loc))
                        (at start (located_at ?content ?loc))
                        (at start (is_empty ?box))
                        (over all (joined ?r ?c)))
        :effect (and (at end (contains ?box ?content))
                     (at end (not (is_empty ?box))))
    )
)
