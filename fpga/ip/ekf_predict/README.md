# fpga/ip/ekf_predict/

**Status:** Concept only, explicitly deferred to Phase 3.

## Purpose

EKF predict step as a Vivado HLS block, generated from the same Jacobian
source CppADCodeGen produces for the software EKF — so the PL and software
implementations stay derived from one model instead of diverging.

## Phase

3

## Hard dependency

`modules/gnc` is currently scaffolding only (`package.xml`, no EKF
implementation, no CppADCodeGen pipeline). This block cannot start until
that exists and is stable — there is nothing to derive the HLS block from
yet.
