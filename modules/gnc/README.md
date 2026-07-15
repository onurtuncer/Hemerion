# modules/gnc/

**Status:** Concept only. `package.xml` exists (declares the
`hemerion_gnc` static-lib/FMU/test artifact triple per `modules/README.md`'s
convention); no `include/`, `src/`, or `test/` exist yet.

## Planned scope

EKF/UKF state estimation and SE(3) control law, per `modules/README.md`'s
module table. This README fixes the EKF design intent referenced
elsewhere in the docs — the `ekf_predict` FPGA IP block (maintained in a
separate repository) has a hard dependency on it.

## EKF design intent

| Parameter | Value |
|---|---|
| State vector | 15-dim: position, velocity, attitude (quaternion), gyro bias, accel bias |
| Predict rate | 500 Hz, driven by IMU samples |
| Measurement update rate | 18 Hz, driven by GPS fixes |
| Jacobian source | Generated on the host via CppADCodeGen, not computed on-target |

## Jacobian generation flow

The Jacobians (`F` for process, `H` for measurement) are derived
symbolically on a developer machine and the generated plain-C output is
checked in — the target firmware has **no CppAD runtime dependency**.

```cpp
// tools/gen_ekf_jacobians.cpp — host-only, not part of the firmware build
#include <cppad/cg.hpp>

using CG  = CppAD::cg::CG<double>;
using AD  = CppAD::AD<CG>;

void generateFMatrix() {
    std::vector<AD> x(15);              // pos/vel/att(quat)/bg/ba
    CppAD::Independent(x);
    std::vector<AD> F = ekfProcessJacobian(x);   // symbolic derivative
    CppAD::ADFun<CG> fun(x, F);

    CppAD::cg::ModelCSourceGen<double> gen(fun, "ekf_jacobian_F");
    gen.setCreateJacobian(true);
    // -> ekf_jacobian_F.c, copied into modules/gnc/src/ekf/
    //    No CppAD dependency in that file.
}
```

`tools/gen_ekf_jacobians.cpp` does not exist yet — `find_package(CppADCodeGen
REQUIRED)` would only be needed by that host-side generator, never by the
`hemerion_gnc` target itself.

## Dependencies once implemented

Per `modules/README.md`'s rules: `FreeRTOS::Kernel`, `ETL::etl`, and the
BSP HAL headers only. No direct CppAD/CppADCodeGen dependency in the
module — that tool only touches the offline generator above.

## Open question

Whether `gnc::EkfCore` and `gnc::Se3Controller` (named in earlier design
notes) survive as the actual class names, or whether the implementation
converges on something else once started — nothing here is fixed besides
the state vector, rates, and the no-CppAD-on-target constraint.
