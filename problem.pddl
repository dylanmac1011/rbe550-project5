(define (problem BLOCKSPROBLEM)
(:domain BLOCKS2)
(:objects r g b y m c - block 
s1 s3 s2 s4 s5 s6 - slot)
(:init (empty s1) (empty s3) (empty s2) (empty s4) (empty s5) (empty s6) (unused r) (unused g) (unused b) (unused y) (unused m) (unused c) (gridempty) (handempty)
(ontable s1) (ontable s2) (ontable s3) (ontable s4) (ontable s5) (ontable s6)
(north s2 s1) (east s3 s1) (northeast s4 s3) (northeast s5 s2) (north s6 s4) (east s6 s5))
(:goal (AND (filled s1) (filled s3) (filled s2) (filled s4) (filled s5) (filled s6) )))
