From mathcomp Require Import all_ssreflect all_algebra.
Require Extraction.
Extraction Language OCaml.

Definition compute_thrust (nd : 'M[nat]_(1,3)) : nat := 1.

Extraction "ThrusterAllocation.ml" compute_thrust.