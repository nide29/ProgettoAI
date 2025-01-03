(define (problem warehouse_problem)
  (:domain manufacturing_temporal)

  (:objects
    robot - robot
    carrier - carrier
    warehouse location1 location2 - location
    workstation1 workstation2 workstation3 - workstation
    box1 box2 box3 - box
    bolt valve - content
    slot1 slot2 slot3 - slot    ; Il carrier ha quindi capacità pari a 3 slot
  )

  (:init
    ; Il robot si trova nella warehouse
    (located_at robot warehouse)
    
    ; Colleghiamo il carrier al robot
    (joined robot carrier)
    
    ; Identifichiamo la warehouse
    (is_warehouse warehouse)

    ; I box sono collocati nella warehouse
    (located_at box1 warehouse)
    (located_at box2 warehouse)
    (located_at box3 warehouse)

    ; I contenuti da mettere nei box sono nella warehouse
    (located_at bolt warehouse)
    (located_at valve warehouse)
    
    ; Le scatole sono vuote
    (is_empty box1)
    (is_empty box2)
    (is_empty box3)
    
    ; Posizioniamo le workstation
    (located_at workstation1 location1)
    (located_at workstation2 location2)
    (located_at workstation3 location2)
    
    ; Connessioni
    (linked warehouse location1)
    (linked location1 location2)
    (linked location1 warehouse)
    (linked location2 location1)
    
    ; Impostiamo la capacità del carrier
    (handle carrier slot1)
    (handle carrier slot2) 
    (handle carrier slot3) 
    (available slot1)
    (available slot2)
    (available slot3)
  )

  (:goal
    (and
      ; Almeno una workstation che necessita di un bullone
      (inside_ws bolt workstation2)

      ; Almeno una workstation che non necessita di nulla
      (not (inside_ws bolt workstation1))
      (not (inside_ws valve workstation1))

      ; Almeno una workstation che necessita di bulloni e viti
      (inside_ws bolt workstation3)
      (inside_ws valve workstation3)
    )
  )
)
