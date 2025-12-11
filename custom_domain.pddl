(define (domain BLOCKS2)
    (:requirements :strips :typing)
    (:types block slot)
    (:predicates 
            ; Straightforward
            (handempty)
            (gridempty)
            (holding ?x - block)
            (unused ?x - block)
            
            ; Slot X is "__" of slot Y
            ; These are STATIC and should NEVER change after being set in :init
            (east ?x - slot ?y - slot)
            (north ?x - slot ?y - slot)
            (northeast ?x - slot ?y - slot)
            (above ?x - slot ?y - slot)

            ; Define for edges of the structure
            (no-north ?s - slot)
            (no-east ?s - slot)
            (no-south ?s - slot)
            (no-west ?s - slot)
            (ontable ?s - slot)

            ; Slot is empty or filled
            (empty ?x - slot)
            (filled ?x - slot)

            ; Block B is in Slot S (1 block per slot max)
            (in ?b - block ?s - slot)
            )

    (:action pick-up
        :parameters (?x - block)
        :precondition (and 
            (handempty)
            (unused ?x))
        :effect (and 
            (not (handempty))
            (holding ?x))
    )

    ; Place block B north of block BREF
    ; slot arguments are pddl internal used to check if we can place a block there
    (:action place-north
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot)
        :precondition (and 
            (holding ?b)
            (unused ?b)
            (filled ?sref)
            (empty ?s)
            (ontable ?s)
            (north ?s ?sref)
            (in ?bref ?sref))
        :effect (and 
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place block B south of block BREF 
    ; Checks that SREF is north of S to accomplish this more simply
    (:action place-south
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot)
        :precondition (and 
            (holding ?b)
            (unused ?b)
            (filled ?sref)
            (empty ?s)
            (ontable ?s)
            (north ?sref ?s)
            (in ?bref ?sref))
        :effect (and 
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place block B east of block BREF
    (:action place-east
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot)
        :precondition (and 
            (holding ?b)
            (unused ?b)
            (filled ?sref)
            (empty ?s)
            (ontable ?s)
            (east ?s ?sref)
            (in ?bref ?sref))
        :effect (and 
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place block B west of block BREF 
    (:action place-west
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot)
        :precondition (and 
            (holding ?b)
            (unused ?b)
            (filled ?sref)
            (empty ?s)
            (ontable ?s)
            (east ?sref ?s)
            (in ?bref ?sref))
        :effect (and 
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place block B northeast of block BREF
    (:action place-northeast
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot)
        :precondition (and 
            (holding ?b)
            (unused ?b)
            (filled ?sref)
            (empty ?s)
            (ontable ?s)
            (northeast ?s ?sref)
            (in ?bref ?sref))
        :effect (and 
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place block B southwest of block BREF 
    (:action place-southwest
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot)
        :precondition (and 
            (holding ?b)
            (unused ?b)
            (filled ?sref)
            (empty ?s)
            (ontable ?s)
            (northeast ?sref ?s)
            (in ?bref ?sref))
        :effect (and 
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place block B above block BREF - Case1 : +
    (:action place-above-1
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot ?srefN - slot ?srefE - slot ?srefS - slot ?srefW - slot)
        :precondition (and
            ; Block can be placed
            (holding ?b)
            (unused ?b)
            (empty ?s)
            (in ?bref ?sref)
            ; Support structure exists
            (filled ?sref)
            (filled ?srefN)
            (filled ?srefS)
            (filled ?srefE)
            (filled ?srefW)
            ; Define slot layout
            (above ?s ?sref)
            (north ?srefN ?sref)
            (east ?srefE ?sref)
            (north ?sref ?srefS)
            (east ?sref ?srefW))
        :effect (and
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place block B above block BREF - Case2 : northT
    (:action place-above-2
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot ?srefE - slot ?srefS - slot ?srefW - slot)
        :precondition (and
            ; Block can be placed
            (holding ?b)
            (unused ?b)
            (empty ?s)
            (in ?bref ?sref)
            ; Support structure exists
            (filled ?sref)
            (no-north ?sref)
            (filled ?srefE)
            (filled ?srefS)
            (filled ?srefW)
            ; Define slot layout
            (above ?s ?sref)
            (east ?srefE ?sref)
            (north ?sref ?srefS)
            (east ?sref ?srefW))
        :effect (and
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place block B above block BREF - Case3 : eastT
    (:action place-above-3
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot ?srefN - slot ?srefS - slot ?srefW - slot)
        :precondition (and
            ; Block can be placed
            (holding ?b)
            (unused ?b)
            (empty ?s)
            (in ?bref ?sref)
            ; Support structure exists
            (filled ?sref)
            (filled ?srefN)
            (no-east ?sref)
            (filled ?srefS)
            (filled ?srefW)
            ; Define slot layout
            (above ?s ?sref)
            (north ?srefN ?sref)
            (north ?sref ?srefS)
            (east ?sref ?srefW))
        :effect (and
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place block B above block BREF - Case4 : southT
    (:action place-above-4
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot ?srefN - slot ?srefE - slot ?srefW - slot)
        :precondition (and
            ; Block can be placed
            (holding ?b)
            (unused ?b)
            (empty ?s)
            (in ?bref ?sref)
            ; Support structure exists
            (filled ?sref)
            (filled ?srefN)
            (filled ?srefE)
            (no-south ?sref)
            (filled ?srefW)
            ; Define slot layout
            (above ?s ?sref)
            (north ?srefN ?sref)
            (east ?srefE ?sref)
            (east ?sref ?srefW))
        :effect (and
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place block B above block BREF - Case5 : westT
    (:action place-above-5
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot ?srefN - slot ?srefE - slot ?srefS - slot)
        :precondition (and
            ; Block can be placed
            (holding ?b)
            (unused ?b)
            (empty ?s)
            (in ?bref ?sref)
            ; Support structure exists
            (filled ?sref)
            (filled ?srefN)
            (filled ?srefE)
            (filled ?srefS)
            (no-west ?sref)
            ; Define slot layout
            (above ?s ?sref)
            (north ?srefN ?sref)
            (east ?srefE ?sref)
            (north ?sref ?srefS))
        :effect (and
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place block B above block BREF - Case6 : vertI
    (:action place-above-6
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot ?srefN - slot ?srefS - slot)
        :precondition (and
            ; Block can be placed
            (holding ?b)
            (unused ?b)
            (empty ?s)
            (in ?bref ?sref)
            ; Support structure exists
            (filled ?sref)
            (filled ?srefN)
            (no-east ?sref)
            (filled ?srefS)
            (no-west ?sref)
            ; Define slot layout
            (above ?s ?sref)
            (north ?srefN ?sref)
            (north ?sref ?srefS))
        :effect (and
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place block B above block BREF - Case7 : horzI
    (:action place-above-7
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot ?srefE - slot ?srefW - slot)
        :precondition (and
            ; Block can be placed
            (holding ?b)
            (unused ?b)
            (empty ?s)
            (in ?bref ?sref)
            ; Support structure exists
            (filled ?sref)
            (no-north ?sref)
            (filled ?srefE)
            (no-south ?sref)
            (filled ?srefW)
            ; Define slot layout
            (above ?s ?sref)
            (east ?srefE ?sref)
            (east ?sref ?srefW))
        :effect (and
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place block B above block BREF - Case8 : northL
    (:action place-above-8
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot ?srefN - slot ?srefE - slot)
        :precondition (and
            ; Block can be placed
            (holding ?b)
            (unused ?b)
            (empty ?s)
            (in ?bref ?sref)
            ; Support structure exists
            (filled ?sref)
            (filled ?srefN)
            (filled ?srefE)
            (no-south ?sref)
            (no-west ?sref)
            ; Define slot layout
            (above ?s ?sref)
            (north ?srefN ?sref)
            (east ?srefE ?sref))
        :effect (and
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place block B above block BREF - Case9 : eastL
    (:action place-above-9
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot ?srefE - slot ?srefS - slot)
        :precondition (and
            ; Block can be placed
            (holding ?b)
            (unused ?b)
            (empty ?s)
            (in ?bref ?sref)
            ; Support structure exists
            (filled ?sref)
            (no-north ?sref)
            (filled ?srefE)
            (filled ?srefS)
            (no-west ?sref)
            ; Define slot layout
            (above ?s ?sref)
            (east ?srefE ?sref)
            (north ?sref ?srefS))
        :effect (and
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place block B above block BREF - Case10 : southL
    (:action place-above-10
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot ?srefS - slot ?srefW - slot)
        :precondition (and
            ; Block can be placed
            (holding ?b)
            (unused ?b)
            (empty ?s)
            (in ?bref ?sref)
            ; Support structure exists
            (filled ?sref)
            (no-north ?sref)
            (no-east ?sref)
            (filled ?srefS)
            (filled ?srefW)
            ; Define slot layout
            (above ?s ?sref)
            (north ?sref ?srefS)
            (east ?sref ?srefW))
        :effect (and
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place block B above block BREF - Case11 : westL
    (:action place-above-11
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot ?srefN - slot ?srefW - slot)
        :precondition (and
            ; Block can be placed
            (holding ?b)
            (unused ?b)
            (empty ?s)
            (in ?bref ?sref)
            ; Support structure exists
            (filled ?sref)
            (filled ?srefN)
            (no-east ?sref)
            (no-south ?sref)
            (filled ?srefW)
            ; Define slot layout
            (above ?s ?sref)
            (north ?srefN ?sref)
            (east ?sref ?srefW))
        :effect (and
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place block B above block BREF - Case12 : northU
    (:action place-above-12
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot ?srefN - slot)
        :precondition (and
            ; Block can be placed
            (holding ?b)
            (unused ?b)
            (empty ?s)
            (in ?bref ?sref)
            ; Support structure exists
            (filled ?sref)
            (filled ?srefN)
            (no-east ?sref)
            (no-south ?sref)
            (no-west ?sref)
            ; Define slot layout
            (above ?s ?sref)
            (north ?srefN ?sref))
        :effect (and
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place block B above block BREF - Case13 : eastU
    (:action place-above-13
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot ?srefE - slot)
        :precondition (and
            ; Block can be placed
            (holding ?b)
            (unused ?b)
            (empty ?s)
            (in ?bref ?sref)
            ; Support structure exists
            (filled ?sref)
            (no-north ?sref)
            (filled ?srefE)
            (no-south ?sref)
            (no-west ?sref)
            ; Define slot layout
            (above ?s ?sref)
            (east ?srefE ?sref))
        :effect (and
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place block B above block BREF - Case14 : southU
    (:action place-above-14
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot ?srefS - slot)
        :precondition (and
            ; Block can be placed
            (holding ?b)
            (unused ?b)
            (empty ?s)
            (in ?bref ?sref)
            ; Support structure exists
            (filled ?sref)
            (no-north ?sref)
            (no-east ?sref)
            (filled ?srefS)
            (no-west ?sref)
            ; Define slot layout
            (above ?s ?sref)
            (north ?sref ?srefS))
        :effect (and
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place block B above block BREF - Case15 : westU
    (:action place-above-15
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot ?srefW - slot)
        :precondition (and
            ; Block can be placed
            (holding ?b)
            (unused ?b)
            (empty ?s)
            (in ?bref ?sref)
            ; Support structure exists
            (filled ?sref)
            (no-north ?sref)
            (no-east ?sref)
            (no-south ?sref)
            (filled ?srefW)
            ; Define slot layout
            (above ?s ?sref)
            (east ?sref ?srefW))
        :effect (and
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place block B above block BREF - Case16 : o
    (:action place-above-16
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot)
        :precondition (and
            ; Block can be placed
            (holding ?b)
            (unused ?b)
            (empty ?s)
            (in ?bref ?sref)
            ; Support structure exists
            (filled ?sref)
            (no-north ?sref)
            (no-east ?sref)
            (no-south ?sref)
            (no-west ?sref)
            ; Define slot layout
            (above ?s ?sref))
        :effect (and
            (not (holding ?b))
            (handempty)
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (in ?b ?s))
    )

    ; Place the first block into a slot
    (:action place-first
        :parameters (?b - block ?s - slot)
        :precondition (and 
            (holding ?b)
            (gridempty)
            (unused ?b)
            (empty ?s)
            (ontable ?s))
        :effect (and 
            (not (gridempty))
            (not (unused ?b))
            (not (empty ?s))
            (filled ?s)
            (handempty)
            (not (holding ?b))
            (in ?b ?s))
    )
)