(define (problem BLOCKSPROBLEM)
(:domain BLOCKS2)
(:objects r g b y m c o w br p - block 
s3 s5 s2 s4 s6 s1 s8 s9 s7 s10 - slot)
(:init (empty s3) (empty s5) (empty s2) (empty s4) (empty s6) (empty s1) (empty s8) (empty s9) (empty s7) (empty s10) (unused r) (unused g) (unused b) (unused y) (unused m) (unused c) (unused o) (unused w) (unused br) (unused p) (gridempty) (handempty)
; Define slot relations
        ; First layer
    (ontable s1) (ontable s2) (ontable s3) (ontable s4) (ontable s5) (ontable s6)
    (north s1 s2) (north s2 s3) (north s4 s5)
    (east s4 s2) (east s5 s3) (east s6 s5)
        ; Second layer
    (above s7 s2) (above s8 s3) (above s9 s5)
    (north s7 s8) (east s9 s8)
        ; Third layer
    (above s10 s8)

    ; Define structure edges
    (no-north s1) (no-east s1) (no-west s1)
    (no-west s2)
    (no-south s3) (no-west s3)
    (no-north s4) (no-east s4)
    (no-south s5)
    (no-north s6) (no-east s6) (no-south s6)
    (no-north s7) (no-east s7) (no-west s7)
    (no-south s8) (no-west s8)
    (no-north s9) (no-east s9) (no-south s9)
    (no-north s10) (no-east s10) (no-south s10) (no-west s10))
(:goal (AND (filled s3) (filled s5) (filled s2) (filled s4) (filled s6) (filled s1) (filled s8) (filled s9) (filled s7) (filled s10) )))
