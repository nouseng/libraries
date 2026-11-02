# Roll2RollDynamics changelog

## 1.1.0 - 2026-11-02

This release bundles Roll2RollDynamics 1.1.0 and Roll2RollDynamicsTest 1.1.0,
adding synchronous belt-drive contact, zero-tension slack cutoff, MultiBody
belt clamps and the `BeltDrivenCarriage` benchmark example.

### 1.1.0 changes

- Add `noSlip` (synchronous kinematic roll contact) and `allowSlack`
  (zero-tension compressive cutoff) to `WebWorld`, `Belt`, `Roller` and
  `Utilities.Parts.WebFriction`.
- Generalize `Components.WebForce` into `Components.WebBoundary`, preserving
  prescribed-force pull (`useSupport = false`) and adding MultiBody frame-clamp
  mode (`useSupport = true`) to anchor belt ends onto moving bodies.
- Add `Examples.BeltDrivenCarriage`, modelling a pre-tensioned Megadyne AT10
  linear belt-drive stage validated against Mehmood et al. (2026) for
  position-dependent span stiffness, dynamic tension sharing and fast-return
  slack cutoff.
- Enforce strict Modelica 3.6 connector and initialization compliance across
  library examples and test suites.
- Update package versions, the Test dependency and the interface version check.

### 1.1.0 validation

The 1.1.0 bundle was validated with OpenModelica 1.27.0 and Modelica Standard
Library 4.1.0 on Windows with `CheckLibrary`, `TestInterfaces`,
`TestAnnotations` and `test_build_images.py`.

## 1.0.1 - 2026-10-23

This patch bundles Roll2RollDynamics 1.0.1 and Roll2RollDynamicsTest 1.0.1.
Existing 1.0.0 models require no conversion. The main library's physical
equations, public interfaces, parameter defaults and experiment settings
are unchanged.

### Changes

- Make Test library component icons visible in diagrams and draw explicit,
  visible, straight connection lines.
- Use the standard Modelica example icon for all 170 Test models and the
  examples-package icon for all 23 Test packages.
- Give every Test model a short explanation of its purpose, including
  fixtures, helper models and scenarios that intentionally reject inputs.
- Run nine simulation probes in build subdirectories to contain generated
  files.
- Update package versions, the Test dependency and the interface version check.

### Validation

The updated Test library was validated with OpenModelica 1.27.0 and Modelica
Standard Library 4.1.0 on Windows before the version metadata update:

- 165 non-partial Test models passed compiler checks; five partial fixtures
  parsed. Seven helper models emitted expected standalone outer-world warnings.
- All 21 regression and structural suites and all 27 supplemental cases passed.
- 131 simulation executions succeeded; seven invalid-input cases were rejected
  as intended. These runs covered 136 distinct Test scenarios.

The 1.0.1 bundle is additionally checked after extraction with CheckLibrary,
TestInterfaces and TestAnnotations. Physics sources are compared with the
validated development revision. This packaging check does not repeat the
simulation suites.

## 1.0.0 - 2026-10-07

Initial public release.
