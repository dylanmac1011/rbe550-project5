(define (problem BLOCKS_PROBLEM)
(:domain BLOCKS2)
(:objects b1 b2 b3 b4 b5 b6 - block
          s1 s2 s3 s4 s5 s6 - slot)
(:init
    (handempty) (gridempty)
    (unused b1) (unused b2) (unused b3) (unused b4) (unused b5) (unused b6)
    (empty s1) (empty s2) (empty s3) (empty s4) (empty s5) (empty s6)
    (north s3 s1) (east s4 s1) (northeast s5 s3) 
    (northeast s6 s4) (north s2 s6) (east s2 s5)
    )
(:goal (and
    (filled s1) (filled s2) (filled s3) (filled s4) (filled s5) (filled s6)))
)
; (:init 
;     (north s3 s1) (east s4 s1) (northeast s5 s3) 
;     (northeast s6 s4) (north s2 s6) (east s2 s5)
;     (handempty) (gridempty)
;     (unused b1) (unused b2) (unused b3) (unused b4) (unused b5) (unused b6)
;     (empty s1) (empty s2) (empty s3) (empty s4) (empty s5) (empty s6))

; (:goal (and
;     (filled s1) (filled s2) (filled s3)
;     (filled s4) (filled s5) (filled s6))))
