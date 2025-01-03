(define (domain gestione_scarti)
  (:requirements :strips :typing :durative-actions :equality :fluents)
  (:types
    location box content robot workstation carrier - object 
  )
  (:predicates
    (atloc ?obj - (either robot workstation box content) ?loc - location)
    (isempty ?box - box)
    (filled ?box - box ?cont - content)
    (atws ?obj - (either robot box content) ?ws - workstation)
    (carrying ?carrier - carrier ?box - box) ;<-modifica rispetto all'istanza 2.1
    ;non c'è più il predicato "free" perchè ora il robot ha il carrier per trasportare, quindi non avrà mai "le mani" occupate nel trasportare una scatola
    (connected ?loc1 ?loc2 - location)
    (iswarehouse ?loc - location)  ;individuiamo una ed una sola location come warehouse, perchè la warehouse ipotizziamo avere infinite risorse
    ;da qui in giù NUOVI PREDICATI rispetto all'istanza 2.1
    (joined ?robot - robot ?carrier - carrier)
  )
  (:functions
    (capacity ?carrier - carrier)
    (load ?carrier - carrier)
  )
  
  ;il robot si muove da una location ad una adiacente (che non sia la warehouse)
  (:durative-action movetoloc
    :parameters (?r - robot ?from - location ?to - location ?car - carrier)
    :duration (= ?duration 3)
    :condition (and 
      (at start(atloc ?r ?from))
      (over all(connected ?from ?to)) 
      (over all(not (iswarehouse ?to))) 
      (over all(joined ?r ?car)))
    :effect (and 
      (at start(not (atloc ?r ?from)))
      (at end(atloc ?r ?to))
    )
  )
  
  ;il robot si muove da una location ad una adiacente (che è la warehouse)
  ;abbiamo bisogno di queste due azioni molto simili, perchè nella warehouse ci possiamo andare solo con il carrier vuoto
  (:durative-action movetowarehouse
    :parameters (?r - robot ?from - location ?to - location ?car - carrier)
    :duration (= ?duration 3)
    :condition (and 
      (at start(atloc ?r ?from)) 
      (over all(connected ?from ?to)) 
      (over all(iswarehouse ?to)) 
      (over all(joined ?r ?car)) 
    )
    :effect (and 
      (at start(not (atloc ?r ?from))) 
      (at end(atloc ?r ?to)))
  )
  
  ;il robot entra in una workstation (facciamo risultare che il robot non è più dentro la location per evitare inconsistenze)
  (:durative-action enterws
    :parameters (?r - robot ?loc - location ?ws - workstation ?car - carrier)
    :duration (= ?duration 0)
    :condition (and 
      (over all(atloc ?ws ?loc)) 
      (at start(atloc ?r ?loc)) 
      (over all(joined ?r ?car)))
    :effect (and 
      (at start(not (atloc ?r ?loc))) 
      (at end(atws ?r ?ws)))
  )
  
  ;il robot esce dalla workstation, ma comunque rimane nella location
  ;notiamo come ciò non crei inconsistenze perchè nella warehouse non ci sono workstation, dunque non rischiamo di far entrare nella warehouse un carrier "pieno"
  (:durative-action exitws
    :parameters (?r - robot ?loc - location ?ws - workstation ?car - carrier)
    :duration (= ?duration 0)
    :condition (and 
      (over all(atloc ?ws ?loc)) 
      (at start(atws ?r ?ws)) 
      (over all(joined ?r ?car))
    )
    :effect (and 
      (at end(atloc ?r ?loc)) 
      (at start(not (atws ?r ?ws)))
    )
  )
  
  ;il robot posa la scatola che sta traportando nel carrier nella workstation in cui si trova
  (:durative-action putdownboxinws
    :parameters (?r - robot ?box - box ?ws - workstation ?car - carrier)
    :duration (= ?duration 2)
    :condition (and 
      (over all (atws ?r ?ws)) 
      (at start (carrying ?car ?box)) 
      (over all (joined ?r ?car))
    )
    :effect (and 
      (at start (not (carrying ?car ?box))) 
      (at end (atws ?box ?ws)) 
      (at end (decrease (load ?car) 1)))
  )
  
  ;il robot posa la scatola che sta traportando nel carrier nella location in cui si trova
  (:durative-action putdownboxinloc
    :parameters (?r - robot ?box - box ?loc - location ?car - carrier)
    :duration (= ?duration 2)
    :condition (and 
      (over all (atloc ?r ?loc)) 
      (at start (carrying ?car ?box)) 
      (over all (joined ?r ?car)))
    :effect (and 
      (at start (not (carrying ?car ?box))) 
      (at end (atloc ?box ?loc)) 
      (at end (decrease (load ?car) 1)))
  )
  
  ;il robot carica una scatola dalla workstation in cui si trova (solo se vuoto)
  (:durative-action loadboxesfromws
    :parameters (?r - robot ?box - box ?ws - workstation ?car - carrier)
    :duration (= ?duration 2)
    :condition (and 
      (at start (atws ?box ?ws)) 
      (over all (atws ?r ?ws)) 
      (over all (joined ?r ?car)) 
      (over all (< (load ?car) (capacity ?car))))
    :effect (and 
      (at end (carrying ?car ?box)) 
      (at end (increase (load ?car) 1)) 
      (at start (not (atws ?box ?ws)))
    )
  )
  
  ;il robot carica una scatola dalla location in cui si trova (solo se vuoto)
  (:durative-action loadboxesfromloc
    :parameters (?r - robot ?box - box ?loc - location ?car - carrier)
    :duration (= ?duration 2)
    :condition (and 
      (at start (atloc ?box ?loc)) 
      (over all (atloc ?r ?loc)) 
      (over all (joined ?r ?car)) 
      (over all (< (load ?car) (capacity ?car))))
    :effect (and 
      (at end (carrying ?car ?box)) 
      (at end (increase (load ?car) 1)) 
      (at start (not (atloc ?box ?loc)))
    )
  )
  
  ;se il carrier non sta caricando, e si trova in una workstation con un box pieno, il robot svuota il box, posizionando il contenuto nella workstation
  (:durative-action emptyboxinws
    :parameters (?r - robot ?box - box ?ws - workstation ?con - content ?car - carrier)
    :duration (= ?duration 3.5)
    :condition (and 
      (over all (atws ?r ?ws)) 
      (at start (atws ?box ?ws)) 
      (at start (filled ?box ?con)) 
      (over all (joined ?r ?car))
    )
    :effect (and 
      (at start (not (filled ?box ?con))) 
      (at end (isempty ?box)) 
      (at end (atws ?con ?ws))
      (at end (atws ?box ?ws))
    )
  )

  ;se il carrier non sta caricando, e si trova in una workstation con un box vuoto, il robot riempie il box prendendo il contenuto dalla workstation
  (:durative-action fillboxinws
    :parameters (?r - robot ?box - box ?ws - workstation ?con - content ?car - carrier)
    :duration (= ?duration 3.5)
    :condition (and 
      (over all (atws ?r ?ws)) 
      (over all (atws ?box ?ws)) 
      (at start (atws ?con ?ws)) 
      (at start (isempty ?box)) 
      (over all (joined ?r ?car))
    )
    :effect (and 
      (at start (not (atws ?con ?ws))) 
      (at end (filled ?box ?con)) 
      (at start (not (isempty ?box))))
  )
  
  ;se il carrier non sta caricando, e si trova in una location con un box pieno, il robot svuota il box, posizionando il contenuto nella location
  (:durative-action emptyboxinloc
    :parameters (?r - robot ?box - box ?con - content ?loc - location ?car - carrier)
    :duration (= ?duration 3.5)
    :condition (and 
      (over all (atloc ?r ?loc)) 
      (at start (atloc ?box ?loc)) 
      (at start (filled ?box ?con)) 
      (over all (joined ?r ?car))
    )
    :effect (and 
      (at start (not (filled ?box ?con))) 
      (at end (isempty ?box)) 
      (at end (atloc ?con ?loc))
    )
  )

  ;se il carrier non sta caricando, e si trova in una location con un box vuoto, il robot riempie il box prendendo il contenuto dalla location
  (:durative-action fillboxinloc
    :parameters (?r - robot ?box - box ?con - content ?loc - location ?car - carrier)
    :duration (= ?duration 3.5)
    :condition (and 
      (over all (not (iswarehouse ?loc))) 
      (over all (atloc ?r ?loc)) 
      (at start (atloc ?box ?loc)) 
      (at start (atloc ?con ?loc)) 
      (at start (isempty ?box)) 
      (over all (joined ?r ?car))
    )
    :effect (and 
      (at start (not (atloc ?con ?loc))) 
      (at end (filled ?box ?con)) 
      (at start (not (isempty ?box))))
  )
  
  ;se il carrier non sta caricando, e si trova nella warehouse con un box vuoto, il robot riempie il box e non consuma alcun oggetto
  (:durative-action fillboxinwarehouse
    :parameters (?r - robot ?box - box ?con - content ?loc - location ?car - carrier)
    :duration (= ?duration 3.5)
    :condition (and 
      (over all (iswarehouse ?loc)) 
      (over all (atloc ?r ?loc)) 
      (over all (atloc ?box ?loc)) 
      (at start (atloc ?con ?loc)) 
      (at start (isempty ?box)) 
      (over all (joined ?r ?car))
    )
    :effect (and 
      (at end (filled ?box ?con)) 
      (at start (not (isempty ?box)))
    )
  )
)