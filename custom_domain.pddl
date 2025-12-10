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
            (east ?x - slot ?y - slot)
            (north ?x - slot ?y - slot)
            (northeast ?x - slot ?y - slot)

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

    ; Place block B north of block BREF
    ; slot arguments are pddl internal used to check if we can place a block there
    (:action place-east
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot)
        :precondition (and 
            (holding ?b)
            (unused ?b)
            (filled ?sref)
            (empty ?s)
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

    ; Place block B south of block BREF 
    ; Checks that SREF is north of S to accomplish this more simply
    (:action place-west
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot)
        :precondition (and 
            (holding ?b)
            (unused ?b)
            (filled ?sref)
            (empty ?s)
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

    ; Place block B north of block BREF
    ; slot arguments are pddl internal used to check if we can place a block there
    (:action place-northeast
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot)
        :precondition (and 
            (holding ?b)
            (unused ?b)
            (filled ?sref)
            (empty ?s)
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

    ; Place block B south of block BREF 
    ; Checks that SREF is north of S to accomplish this more simply
    (:action place-southwest
        :parameters (?b - block ?bref - block ?s - slot ?sref - slot)
        :precondition (and 
            (holding ?b)
            (unused ?b)
            (filled ?sref)
            (empty ?s)
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

    ; Place the first block into a slot
    (:action place-first
        :parameters (?b - block ?s - slot)
        :precondition (and 
            (holding ?b)
            (gridempty)
            (unused ?b)
            (empty ?s))
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