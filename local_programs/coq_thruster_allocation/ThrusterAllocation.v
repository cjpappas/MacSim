Require Import Nat.

Definition thruster_allocation (lat : nat) : nat := lat.

Example test_trhuster_allocation_1: (thruster_allocation 1) = 1.
Proof. reflexivity. Qed.