# Roll2RollDynamicsTest validation

All existing scenarios live in the companion library
[`Roll2RollDynamicsTest`](package.mo). In OMEdit, open
`Roll2RollDynamics/package.mo`, then `Roll2RollDynamicsTest/package.mo`.
The library tree groups checks by scenario family, including the figure sweeps.
Partial models are shared fixtures; invalid-input scenarios intentionally fail.

The release review environment is Windows
with OpenModelica 1.27.0, PowerShell 7.6.6 and Python 3.14.4; Linux and macOS
have not been verified. These versions identify the review environment, not
a claim that every suite has passed.

`TestSimulation.ps1` and `TestInterfaces.ps1` pass with Windows PowerShell
5.1.26100.9444. The interface suite also passes with PowerShell 7.6.6.
These results cover those wrappers only.

For example, the runout scenario is now
`Roll2RollDynamicsTest.RunoutChecks.RunoutIdler`, and the standalone friction
check is `Roll2RollDynamicsTest.TripleSFrictionCheck`. The runners below
retain their simulation settings, expected-failure checks and result filenames.
Use them for verification that also requires numerical post-processing.

Run commands from the repository root with OpenModelica on `PATH`, or use
the full path to `omc`. PowerShell test scripts also detect `OPENMODELICAHOME`
and standard Windows installations. These wrappers work from any current
working directory; direct `omc` commands require the repository root.

For routine changes, start with:

```powershell
omc Roll2RollDynamicsTest/Resources/Scripts/CheckLibrary.mos
```

Expect 22 successful structural checks and no errors or warnings. This covers
the public components, six focused examples, the `TangencyQuadrants` test model and two examples in
`Examples.WebLines`, without building simulation executables. Structural
checks do not verify dynamics.

Each example defines standard
preferred result figures; automatic display depends on the Modelica viewer.

After physics changes, run the relevant suite rather than every script.
Set this path in PowerShell from the repository root:

```powershell
$scripts = './Roll2RollDynamicsTest/Resources/Scripts'
```

| Change | Command |
| --- | --- |
| Belt acceleration, moving boundaries | `& "$scripts/TestBeltDynamics.ps1"` |
| Belt damping and defaults | `omc "$scripts/BeltDampingChecks.mos"` |
| Lateral steering and reversal | `& "$scripts/TestLateralSteering.ps1"` |
| Shelton analytical steering benchmark | `& "$scripts/TestShelton.ps1"` |
| Wrap geometry and force routing | `& "$scripts/TestWebWrap.ps1"` |
| Inventory and winding startup | `& "$scripts/TestMassConservation.ps1"` |
| Friction law | `& "$scripts/TestTripleS.ps1"` |
| Nip contact law | `& "$scripts/TestNipContact.ps1"` |
| Web-face traction selection | `omc "$scripts/WebFaceChecks.mos"` |
| Roller web detachment | `& "$scripts/TestDetachment.ps1"` |
| Nip assembly and friction ownership | `& "$scripts/TestNipIntegration.ps1"` |
| Skewed, wedged and eccentric nips | `& "$scripts/TestSkewNip.ps1"` |
| Out-of-plane roll tilt (tram) | `& "$scripts/TestTram.ps1"` |
| Roller drum runout | `& "$scripts/TestRunout.ps1"` |
| Bearing and support reactions | `& "$scripts/TestSupportReaction.ps1"` |
| Assembled line startup and dancer motion | `& "$scripts/TestSimulation.ps1"` |
| Rotation sign | `& "$scripts/TestRotationDirectionValidation.ps1"` |
| Ports and component composition | `& "$scripts/TestInterfaces.ps1"` |
| Icons and diagrams | `& "$scripts/TestAnnotations.ps1"` |
| Animation geometry | `& "$scripts/TestVisualGeometry.ps1"` |

`TestRunout.ps1` runs 11 simulations: a true drum, three runout
magnitudes, a turned runout phase, the folded-in and run-time parameter paths,
a run-time path translated at zero runout and set by `-override` (this is
what an exported FMU does), two eccentric closed loops, an eccentric driven roll
under a PI speed loop and one held nip. It then checks the settled spectra
with the adjacent `check_runout_spectrum.py`. The free span swings by
0.283 mm against a 0.3 mm runout and changes length at 7.57 mm/s against the
`e*omega` its eccentricity and turning rate predict, both from the span
equations of Branca, Pagilla and Reid (2011);
the ripple stays proportional to the runout to within 0.02 percent over 0.075,
0.15 and 0.3 mm; `runoutPhase` turns the ripple one-for-one, 0.799 rad measured
against 0.8 rad set; the closed loop keeps its mass and shares the ripple with
the span that never touches the eccentric roller; and an in-plane orbit never
moves the web across the machine. One measured finding is worth reading before
changing the runout physics: the tension ripple is a clean fundamental, with a
second harmonic at 0.3 percent for a dragged idler and below 0.1 percent for
the PI-driven roll of `DrivenRunout`, while the published line reports
harmonics of 10 to 30 percent. Angular runout (a cocked drum axis)
was removed from the library on 2026-09-16 because no published measurement of
its effect on the web was found to check it against.

`TestMassConservation.ps1` runs 16 simulations. Delayed winding startup covers
both standstill and forward transport; stopped and reverse cases remain
separate. Oblique stretching and reverse-loop tests reuse their base assertions.
The winding fixture lives only in the test package.

`TestShelton.ps1` compares positive yaw, negative yaw and half-speed runs
with the independent first-order prediction. It checks tracking error,
normal entry, running speed and open material balance. Benchmark dimensions
are chosen settings; these checks do not claim experimental validation.

`TestDetachment.ps1` runs `DetachmentChecks.mos` and validates seven scenarios:
six successful simulations and the intended `NipConflict` failure. The latter
checks that combining `useDetachment` with `useNip` is rejected, with an empty
result file and the expected diagnostic. Use the wrapper to check these
outcomes; a zero exit code from `omc` alone does not establish success.
The successful models can also emit transient assertions when a roll releases
or re-engages; OpenModelica supersedes them with "Found event, previous asserts
are ignored". The wrapper checks the final simulation outcomes.
Successful scenarios must finish without compiler errors or warnings.
Diagnostics remain in `build/detachment-checks/verification.log`, including
the intended rejection and ignored event assertions.

`ExamplePhysicsChecks.mos` runs five slip-capacity, idler, steering
and capstan checks. Use it when changing those effects or preparing a release.
It does not repeat the dedicated friction and nip suites.

`BeltDampingChecks.mos` runs seven simulations covering viscous force, creep,
energy, reverse transport, damped and elastic ring-down, and elastic loop
conservation. Ring-down inherits `world.webDampingTime`; uniform extension
checks that a belt can override the line default.

Two figures overlay published measurements kept in
`Roll2RollDynamics/Resources/Data`: the Roller runout spectrum against the
Euclid Web Line spectra of Branca et al. (2011), read from the paper's vector
graphics, and the SheltonSteering yaw response against the nine PET-film
cases of Yun et al. (2025).

## Compiler artifact cleanup

Run the cleanup helper from any working directory. It previews ignored compiler
artifacts at the repository root and under `build/`; add `-Delete` to remove
the enumerated files. It preserves simulation results, metadata and authored
sources.

```powershell
./Roll2RollDynamicsTest/Resources/Scripts/clean_artifacts.ps1
./Roll2RollDynamicsTest/Resources/Scripts/clean_artifacts.ps1 -Delete
```
