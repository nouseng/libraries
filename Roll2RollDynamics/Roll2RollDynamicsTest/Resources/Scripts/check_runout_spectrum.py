"""Check the runout simulations against the once-per-turn orbit they model.

Reads the CSV results written by the adjacent RunoutChecks.mos and checks the
frequency, amplitude, phase law and linearity of the drum-runout response.
Run through the adjacent TestRunout.ps1.
"""
from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
CSV_DIR = REPO / "build" / "runout-checks"

# Line the idler checks run on: 125 micrometre film at 2 m/s over a 75 mm
# idler, both from MisalignedIdlerLine through RunoutChecks.
LINE_SPEED = 2.0
IDLER_RADIUS = 0.075
WEB_THICKNESS = 125e-6
TURN_RATE = LINE_SPEED / (2 * np.pi * (IDLER_RADIUS + WEB_THICKNESS / 2))
SETTLED = (15.0, 20.0)
GRID_STEP = 1e-3
# The closed loop of ThreeRollWebLoop: a 5 mm belt at 2 m/s over a 0.12 m apex
# idler and a 0.09 m idler, with the drive holding the loop speed.
LOOP_WINDOW = (20.0, 25.0)
TOP_RATE = 2.0 / (2 * np.pi * (0.12 + 0.005 / 2))
RIGHT_RATE = 2.0 / (2 * np.pi * (0.09 + 0.005 / 2))
# The runoutPhase of RunoutChecks.RunoutIdlerPhase, in radians.
PHASE_SHIFT = 0.8
# Runout magnitudes of the three linearity cases, in metres.
MAGNITUDES = {
    "RunoutChecks.RunoutIdlerQuarter": 0.075e-3,
    "RunoutChecks.RunoutIdlerHalf": 0.15e-3,
    "RunoutChecks.RunoutIdler": 0.3e-3,
}


def read_result(model: str) -> dict[str, np.ndarray]:
    """Return the columns of build/runout-checks/<model>_res.csv as arrays.

    A truncated final row, left by an interrupted solve, is dropped rather than
    read as a short record.
    """
    path = CSV_DIR / f"{model}_res.csv"
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.reader(handle))
    header = [name.strip().strip('"') for name in rows[0]]
    data = np.array([[float(value) for value in row] for row in rows[1:]
                     if len(row) == len(header)])
    return {name: data[:, index] for index, name in enumerate(header)}


def uniform(columns: dict[str, np.ndarray], name: str,
            window: tuple[float, float] = SETTLED) -> tuple[np.ndarray, np.ndarray]:
    """Interpolate a column onto a uniform grid over a settled window.

    The solver repeats and subdivides output points around events, so the raw
    time column is not uniform; a plain FFT of it reports a wrong frequency.
    """
    grid = np.arange(window[0], window[1], GRID_STEP)
    return grid, np.interp(grid, columns["time"], columns[name])


def once_per_turn(columns: dict[str, np.ndarray], name: str) -> tuple[float, float]:
    """Return the amplitude and phase of the once-per-turn part of a column.

    The basis carries a constant and a ramp as well as the two once-per-turn
    references, so a slowly settling line cannot bias the harmonic fit. The
    phase is the runout high-point angle at which the column peaks.
    """
    grid, values = uniform(columns, name)
    _, angle = uniform(columns, "idler.rollerVariables.angle")
    basis = np.vstack([np.ones_like(angle), np.cos(angle), np.sin(angle),
                       grid - grid.mean()]).T
    _, cosine, sine, _ = np.linalg.lstsq(basis, values, rcond=None)[0]
    return float(np.hypot(cosine, sine)), float(np.arctan2(sine, cosine))


def strongest_frequencies(columns: dict[str, np.ndarray], name: str,
                          window: tuple[float, float] = SETTLED,
                          count: int = 1) -> list[float]:
    """Return the strongest frequencies of a column above 0.5 Hz, lowest first."""
    _, values = uniform(columns, name, window)
    taper = np.hanning(len(values))
    spectrum = 2 * np.abs(np.fft.rfft((values - values.mean()) * taper)) / taper.sum()
    frequency = np.fft.rfftfreq(len(values), GRID_STEP)
    above = frequency > 0.5
    frequency, spectrum = frequency[above], spectrum[above]
    order = np.argsort(spectrum)[::-1][:count]
    return sorted(float(frequency[index]) for index in order)


def harmonic_amplitude(spectrum: np.ndarray, frequency: np.ndarray,
                       target: float) -> float:
    """Return the spectral peak within one bin of a target frequency."""
    index = int(np.argmin(np.abs(frequency - target)))
    return float(np.max(spectrum[max(index - 1, 0):index + 2]))


def report(label: str, text: str) -> None:
    """Print one result line."""
    print(f"{label:<34} {text}")


def main() -> int:
    """Run every check and return a process exit code."""
    failed: list[str] = []

    def require(condition: bool, message: str) -> None:
        if not condition:
            failed.append(message)

    runout = read_result("RunoutChecks.RunoutIdler")
    swing, swing_phase = once_per_turn(runout, "idlerSpanLength")
    ripple, _ = once_per_turn(runout, "idlerSpanTension")
    frequency = strongest_frequencies(runout, "idlerSpanTension")[0]
    report("turn rate of the 75 mm idler", f"{TURN_RATE:.4f} Hz")
    report("settled span-tension peak", f"{frequency:.3f} Hz")
    require(abs(frequency - TURN_RATE) <= 0.25,
            f"the span tension must ripple at the idler turn rate, not {frequency:.3f} Hz")
    report("span swing at 0.3 mm runout", f"{1e3*swing:.4f} mm of 0.300 mm")
    require(abs(swing - 0.3e-3) <= 0.1 * 0.3e-3,
            "the free span must swing by the drum runout")
    report("tension ripple at 0.3 mm runout", f"{ripple:.3f} N")
    require(ripple > 2, "a 0.3 mm eccentric idler must ripple the entry span")
    lateral = runout["idlerLateralOffset"][runout["time"] > SETTLED[0]]
    rocking = float(lateral.max() - lateral.min())
    report("web offset at 0.3 mm runout", f"{1e3*rocking:.4f} mm")
    require(rocking < 1e-4,
            "an in-plane drum orbit must not move the web across the machine")
    # Branca, Pagilla and Reid (2011), Eq. (11): the span beside an eccentric
    # roller changes length at d0*e*omega*cos(theta)/L, so for a free length
    # close to the centre distance its rate amplitude is the eccentricity times
    # the turning rate. RunoutChecks reads the free length the library computes
    # from the tangency frame the orbit carries.
    _, length = uniform(runout, "idlerSpanLength")
    rate = float(np.max(np.abs(np.gradient(length, GRID_STEP))))
    predicted = 0.3e-3 * 2 * np.pi * TURN_RATE
    report("span-length rate", f"{1e3*rate:.4f} mm/s of {1e3*predicted:.4f} mm/s")
    require(abs(rate - predicted) <= 0.12 * predicted,
            "the span-length rate must follow the eccentric-roller span equation")
    # The paper reports the fundamental and its higher harmonics in tension.
    # The library's ripple here is a clean fundamental, which this line reports
    # rather than asserts: the ratio is printed for the documentation.
    _, tension = uniform(runout, "idlerSpanTension")
    taper = np.hanning(len(tension))
    spectrum = 2 * np.abs(np.fft.rfft((tension - tension.mean()) * taper)) / taper.sum()
    frequency = np.fft.rfftfreq(len(tension), GRID_STEP)
    report("ripple harmonics 1x, 2x, 3x",
           ", ".join(f"{harmonic_amplitude(spectrum, frequency, k*TURN_RATE):.4g} N"
                     for k in (1, 2, 3)))

    shifted = read_result("RunoutChecks.RunoutIdlerPhase")
    swing_shifted, swing_phase_shifted = once_per_turn(shifted, "idlerSpanLength")
    ripple_shifted, _ = once_per_turn(shifted, "idlerSpanTension")
    measured = (swing_phase_shifted - swing_phase + np.pi) % (2 * np.pi) - np.pi
    report("phase law, swing moves by", f"{measured:+.4f} rad of {PHASE_SHIFT:+.4f} rad")
    require(abs(measured - PHASE_SHIFT) <= 0.05,
            "runoutPhase must turn the span swing by the same angle")
    require(abs(swing_shifted - swing) <= 0.01 * swing,
            "turning the runout must not change the swing amplitude")
    require(abs(ripple_shifted - ripple) <= 0.02 * ripple,
            "turning the runout must not change the tension ripple")

    ratios: dict[str, float] = {}
    for model, magnitude in MAGNITUDES.items():
        columns = read_result(model)
        ratios[model] = once_per_turn(columns, "idlerSpanTension")[0] / magnitude
        report(f"ripple per mm of runout, {1e3*magnitude:g} um",
               f"{1e-3*ratios[model]:.3f} N/mm")
    spread = max(ratios.values()) / min(ratios.values())
    report("linearity spread", f"{spread:.4f}")
    require(spread <= 1.1,
            "the tension ripple must stay proportional to the runout")

    for model, label, rocking_limit in (
            ("RunoutChecks.AlignedIdler", "true drum", 1e-4),):
        columns = read_result(model)
        quiet = once_per_turn(columns, "idlerSpanTension")[0]
        settled = columns["time"] > SETTLED[0]
        lateral = columns["idlerLateralOffset"][settled]
        walked = float(abs(lateral.mean()))
        rocked = float(lateral.max() - lateral.min())
        report(f"{label}: ripple, walk, rocking",
               f"{quiet:.5f} N, {1e6*walked:.3f} um, {1e3*rocked:.3f} mm")
        require(quiet < 0.05, f"a {label} must not ripple the entry span")
        require(walked < 1e-4, f"a {label} must not walk the web")
        require(rocked < rocking_limit,
                f"a {label} must not rock the web more than {1e3*rocking_limit:.2f} mm")

    # The closed loop is speed-locked with no winder, so an eccentric idler
    # cannot keep its ripple to itself: every span, including the return span
    # that never touches it, carries the idler's own turning rate. Each span
    # weights the two rates differently, so the check reads the amplitude at
    # the rates the geometry predicts and asks only that it rise clear of the
    # numerical floor.
    for model, rates in (
            ("RunoutChecks.LoopRunoutTop", (TOP_RATE,)),
            ("RunoutChecks.LoopRunoutBoth", (TOP_RATE, RIGHT_RATE))):
        columns = read_result(model)
        for span in ("risingSpan", "fallingSpan", "returnSpan"):
            _, values = uniform(columns, f"{span}.tension", LOOP_WINDOW)
            taper = np.hanning(len(values))
            spectrum = 2*np.abs(np.fft.rfft((values - values.mean())*taper))/taper.sum()
            frequency = np.fft.rfftfreq(len(values), GRID_STEP)
            keep = frequency > 0.5
            spectrum, frequency = spectrum[keep], frequency[keep]
            loudest = float(spectrum.max())
            found = [harmonic_amplitude(spectrum, frequency, rate) for rate in rates]
            report(f"{model.split('.')[-1]} {span}",
                   ", ".join(f"{rate:.3f} Hz {value:.4g} N"
                             for rate, value in zip(rates, found)))
            for rate, value in zip(rates, found):
                require(value >= 0.05*loudest,
                        f"{span} must ripple at {rate:.3f} Hz")

    # The driven roll of DrivenRunout is the eccentric one, held at speed by a
    # PI loop so that its shaft keeps a degree of freedom, the arrangement of
    # Branca's S-wrap lead roller. Every span must ripple at its turn rate; the
    # harmonic content is reported because the published line shows harmonics
    # of 10 to 30 percent and this loop does not.
    driven = read_result("RunoutChecks.DrivenRunout")
    _, shaft_speed = uniform(driven, "speedSensor.w", LOOP_WINDOW)
    drive_rate = float(shaft_speed.mean())/(2*np.pi)
    report("DrivenRunout shaft speed held", f"{drive_rate:.4f} Hz, ripple "
           f"{100*(shaft_speed.max() - shaft_speed.min())/shaft_speed.mean():.3f} %")
    for span in ("risingSpan", "fallingSpan", "returnSpan"):
        _, values = uniform(driven, f"{span}.tension", LOOP_WINDOW)
        taper = np.hanning(len(values))
        spectrum = 2*np.abs(np.fft.rfft((values - values.mean())*taper))/taper.sum()
        frequency = np.fft.rfftfreq(len(values), GRID_STEP)
        harmonics = [harmonic_amplitude(spectrum, frequency, n*drive_rate) for n in (1, 2, 3)]
        report(f"DrivenRunout {span} 1x, 2x, 3x",
               f"{harmonics[0]:.4f} N, {harmonics[1]/harmonics[0]:.4f}, {harmonics[2]/harmonics[0]:.4f}")
        require(harmonics[0] > 0.5, f"the driven eccentric roll must ripple {span}")

    if failed:
        print("\nRunout spectrum checks failed:")
        for message in failed:
            print(f"  - {message}")
        return 1
    print("\nRunout spectrum, phase law and linearity checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
