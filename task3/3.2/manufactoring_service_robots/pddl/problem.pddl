(define (problem warehouse_problem)
  (:domain gestione_scarti)

  (:objects
    robot - robot
    carrier - carrier
    warehouse location1 location2 - location
    workstation1 workstation2 workstation3 - workstation
    box1 box2 box3 - box
    bolt screw - content
  )

  (:init
    ; Il robot si trova nella warehouse
    (atloc robot warehouse)
    
    ;colleghiamo il carrier al robot
    (joined robot carrier)
    
    ;identifichiamo la warehouse
    (iswarehouse warehouse)

    ; I box sono collocati nella warehouse
    (atloc box1 warehouse)
    (atloc box2 warehouse)
    (atloc box3 warehouse)

    ; I contenuti da mettere nei box sono nella warehouse
    (atloc bolt warehouse)
    (atloc screw warehouse)
    
    ;le scatole sono vuote
    (isempty box1)
    (isempty box2)
    (isempty box3)
    
    ;posizioniamo le workstation
    (atloc workstation1 location1)
    (atloc workstation2 location2)
    (atloc workstation3 location2)
    
    ;connessioni
    (connected warehouse location1)
    (connected location1 location2)
    (connected location1 warehouse)
    (connected location2 location1)
    
    ; Inizializzazione delle funzioni
    (= (capacity carrier) 2)  ; Capacità iniziale del carrier
    (= (load carrier) 0)      ; Carico iniziale del carrier
  )

  (:goal
    (and
      ; Almeno una workstation che necessita di un bullone
      (atws bolt workstation2)

      ; Almeno una workstation che non necessita di nulla
      (not (atws bolt workstation1))
      (not (atws screw workstation1))

      ; Almeno una workstation che necessita di bulloni e viti
      (atws bolt workstation3)
      (atws screw workstation3)

    )
  )
)