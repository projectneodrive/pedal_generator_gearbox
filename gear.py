import itertools
import math
import os
from dataclasses import dataclass
from functools import lru_cache
from math import gcd


@dataclass(frozen=True)
class SearchConfig:
    gears: tuple[int, ...] = (60, 12, 25, 38, 70, 21, 18)
    min_stages: int = 2
    max_stages: int = 4
    target_ratio_min: float = 58.0
    target_ratio_max: float = 80.0
    top_n_results: int = 12
    allow_gear_reuse: bool = True

    # Backdrivability preferences
    preferred_stage_ratio: float = 3.0
    hard_stage_ratio_limit: float = 4.5

    # Efficiency model (heuristic)
    base_mesh_efficiency: float = 0.975
    ratio_efficiency_penalty: float = 0.010
    min_stage_efficiency: float = 0.90

    # Wear model (heuristic)
    min_pinion_teeth_preferred: int = 17

    # Weighted objective (higher is better)
    w_backdrive: float = 0.40
    w_efficiency: float = 0.30
    w_wear: float = 0.20
    w_cost_mass: float = 0.10

    # Generator + battery model for SOC/RPM efficiency plot
    motor_kv_rpm_per_volt: float = 100.0
    motor_kv_candidates: tuple[float, ...] = (15, 75.0, 100.0, 110.0, 200)
    battery_series_cells: int = 15
    controller_regen_efficiency: float = 0.95
    boost_regen_efficiency: float = 0.92

    # Plot sweep ranges
    rpm_min: int = 40
    rpm_max: int = 120
    rpm_step: int = 2
    soc_min: float = 0.05
    soc_max: float = 1.00
    soc_points: int = 60

    # Output
    plot_file: str = "best_match_efficiency_map.png"
    kv_comparison_plot_file: str = "best_match_efficiency_map_kv_compare.png"


@lru_cache(maxsize=None)
def is_prime(value: int) -> bool:
    if value < 2:
        return False
    if value % 2 == 0:
        return value == 2
    limit = int(math.sqrt(value))
    for number in range(3, limit + 1, 2):
        if value % number == 0:
            return False
    return True


def stage_pairs(gears: tuple[int, ...]) -> list[tuple[int, int]]:
    """
    Stage pair is (driver, driven).
    Per-stage speed ratio = driver / driven.
    """
    return [(driver, driven) for driver, driven in itertools.permutations(gears, 2)]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


def stage_ratio(stage: tuple[int, int]) -> float:
    driver, driven = stage
    return driver / driven


def stage_efficiency(ratio: float, cfg: SearchConfig) -> float:
    ratio_penalty = cfg.ratio_efficiency_penalty * abs(ratio - 1.0)
    eff = cfg.base_mesh_efficiency - ratio_penalty
    return clamp(eff, cfg.min_stage_efficiency, 0.995)


def stage_wear_score(stage: tuple[int, int], cfg: SearchConfig) -> float:
    driver, driven = stage

    score = 0.0

    # Favor relatively-prime mesh (hunting tooth behavior)
    if gcd(driver, driven) == 1:
        score += 1.0

    # Slight bonus if either gear tooth count is prime
    if is_prime(driver):
        score += 0.25
    if is_prime(driven):
        score += 0.25

    # Penalize small pinions (undercut + wear sensitivity)
    pinion = min(driver, driven)
    if pinion < cfg.min_pinion_teeth_preferred:
        score -= 1.0 + 0.1 * (cfg.min_pinion_teeth_preferred - pinion)

    return score


def normalize(value: float, low: float, high: float) -> float:
    if high <= low:
        return 0.0
    return clamp((value - low) / (high - low), 0.0, 1.0)


def score_configuration(stages: tuple[tuple[int, int], ...], cfg: SearchConfig) -> dict:
    ratios = [stage_ratio(stage) for stage in stages]
    total_ratio = math.prod(ratios)

    if total_ratio < cfg.target_ratio_min or total_ratio > cfg.target_ratio_max:
        return {}

    if any(r > cfg.hard_stage_ratio_limit for r in ratios):
        return {}

    mean_ratio = sum(ratios) / len(ratios)
    ratio_spread = max(ratios) - min(ratios)

    # Backdrivability: prefer low and balanced per-stage ratios
    ratio_target_error = abs(mean_ratio - cfg.preferred_stage_ratio)
    backdrive_raw = 1.0 / (1.0 + ratio_target_error + 0.7 * ratio_spread)

    # Efficiency: product of stage efficiencies
    total_efficiency = math.prod(stage_efficiency(r, cfg) for r in ratios)

    # Wear: average of per-stage wear scores
    wear_raw = sum(stage_wear_score(stage, cfg) for stage in stages) / len(stages)

    # Cost/mass: fewer stages is better
    if cfg.max_stages == cfg.min_stages:
        cost_mass_raw = 1.0
    else:
        cost_mass_raw = 1.0 - normalize(len(stages), cfg.min_stages, cfg.max_stages)

    # Normalize wear into [0,1] with a practical bounds window
    wear_norm = normalize(wear_raw, -2.5, 1.5)

    total_score = (
        cfg.w_backdrive * backdrive_raw
        + cfg.w_efficiency * total_efficiency
        + cfg.w_wear * wear_norm
        + cfg.w_cost_mass * cost_mass_raw
    )

    return {
        "stages": stages,
        "ratios": ratios,
        "total_ratio": total_ratio,
        "backdrive": backdrive_raw,
        "efficiency": total_efficiency,
        "wear": wear_raw,
        "cost_mass": cost_mass_raw,
        "score": total_score,
    }


def search(cfg: SearchConfig) -> list[dict]:
    pairs = stage_pairs(cfg.gears)
    results: list[dict] = []

    for n_stages in range(cfg.min_stages, cfg.max_stages + 1):
        for stages in itertools.product(pairs, repeat=n_stages):
            if not cfg.allow_gear_reuse:
                used = [gear for stage in stages for gear in stage]
                if len(set(used)) != len(used):
                    continue

            scored = score_configuration(stages, cfg)
            if scored:
                results.append(scored)

    results.sort(key=lambda item: item["score"], reverse=True)
    return results


def format_stage(stage: tuple[int, int], ratio: float) -> str:
    driver, driven = stage
    return f"{driver}->{driven} (x{ratio:.3f})"


def lifepo4_cell_ocv_from_soc(soc: float) -> float:
    """
    Approximate LiFePO4 open-circuit voltage (per cell) vs SOC.
    Piecewise-linear fit suitable for high-level drivetrain tradeoff analysis.
    """
    points = [
        (0.00, 2.80),
        (0.10, 3.05),
        (0.20, 3.18),
        (0.50, 3.27),
        (0.80, 3.32),
        (0.90, 3.36),
        (1.00, 3.60),
    ]

    soc = clamp(soc, 0.0, 1.0)
    if soc <= points[0][0]:
        return points[0][1]
    if soc >= points[-1][0]:
        return points[-1][1]

    for (x0, y0), (x1, y1) in zip(points[:-1], points[1:]):
        if x0 <= soc <= x1:
            t = (soc - x0) / (x1 - x0)
            return y0 + t * (y1 - y0)

    return points[-1][1]


def linspace(start: float, stop: float, count: int) -> list[float]:
    if count <= 1:
        return [start]
    step = (stop - start) / (count - 1)
    return [start + i * step for i in range(count)]


def regen_transfer_factor(back_emf_v: float, bus_v: float, cfg: SearchConfig) -> float:
    """
    Electrical transfer factor from machine to battery:
    - If EMF >= bus: direct regen path.
    - If EMF < bus: boost-mode regen limited roughly by EMF/bus and switching losses.
    """
    if bus_v <= 0:
        return 0.0

    if back_emf_v >= bus_v:
        return 0.98

    ratio = back_emf_v / bus_v
    return cfg.boost_regen_efficiency * ratio


def build_efficiency_map(best: dict, cfg: SearchConfig, motor_kv: float | None = None) -> dict:
    total_ratio = best["total_ratio"]
    mech_eff = best["efficiency"]
    kv = motor_kv if motor_kv is not None else cfg.motor_kv_rpm_per_volt

    pedal_rpms = list(range(cfg.rpm_min, cfg.rpm_max + 1, cfg.rpm_step))
    socs = linspace(cfg.soc_min, cfg.soc_max, cfg.soc_points)

    z_values: list[list[float]] = []

    for rpm in pedal_rpms:
        motor_rpm = rpm * total_ratio
        back_emf_v = motor_rpm / kv

        row: list[float] = []
        for soc in socs:
            cell_v = lifepo4_cell_ocv_from_soc(soc)
            bus_v = cell_v * cfg.battery_series_cells

            transfer = regen_transfer_factor(back_emf_v, bus_v, cfg)
            regen_eff = mech_eff * cfg.controller_regen_efficiency * transfer
            row.append(clamp(regen_eff, 0.0, 1.0))

        z_values.append(row)

    return {
        "pedal_rpms": pedal_rpms,
        "socs": socs,
        "efficiency": z_values,
        "kv": kv,
    }


def import_matplotlib() -> tuple[object, object] | tuple[None, None]:
    try:
        import matplotlib

        if os.environ.get("DISPLAY", "") == "":
            matplotlib.use("Agg")

        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is not installed; skipping plot generation.")
        print("Install with: pip install matplotlib")
        return None, None

    return matplotlib, plt


def plot_efficiency_map(best: dict, cfg: SearchConfig, motor_kv: float | None = None, file_name: str | None = None) -> str | None:
    _, plt = import_matplotlib()
    if plt is None:
        return None

    kv = motor_kv if motor_kv is not None else cfg.motor_kv_rpm_per_volt
    out_file = file_name if file_name else cfg.plot_file

    data = build_efficiency_map(best, cfg, motor_kv=kv)
    pedal_rpms = data["pedal_rpms"]
    socs = data["socs"]
    z = data["efficiency"]

    fig, ax = plt.subplots(figsize=(9, 5.5))
    image = ax.imshow(
        z,
        origin="lower",
        aspect="auto",
        extent=[socs[0] * 100, socs[-1] * 100, pedal_rpms[0], pedal_rpms[-1]],
        vmin=0.0,
        vmax=1.0,
        cmap="viridis",
    )

    cb = plt.colorbar(image, ax=ax)
    cb.set_label("Estimated regen efficiency (0..1)")

    ax.set_title(
        f"Best Gear Match Regen Efficiency | Kv={kv:.0f} | ratio={best['total_ratio']:.2f}"
    )
    ax.set_xlabel("Battery SOC (%)")
    ax.set_ylabel("Pedaling RPM")

    plt.tight_layout()
    plt.savefig(out_file, dpi=160)
    plt.close(fig)
    return out_file


def plot_efficiency_map_kv_comparison(best: dict, cfg: SearchConfig) -> str | None:
    _, plt = import_matplotlib()
    if plt is None:
        return None

    kv_values = cfg.motor_kv_candidates
    if len(kv_values) == 0:
        return None

    fig, axes = plt.subplots(
        1,
        len(kv_values),
        figsize=(5.2 * len(kv_values), 5.2),
        sharey=True,
        constrained_layout=True,
    )
    if len(kv_values) == 1:
        axes = [axes]

    mappable = None

    for ax, kv in zip(axes, kv_values):
        data = build_efficiency_map(best, cfg, motor_kv=kv)
        pedal_rpms = data["pedal_rpms"]
        socs = data["socs"]
        z = data["efficiency"]

        image = ax.imshow(
            z,
            origin="lower",
            aspect="auto",
            extent=[socs[0] * 100, socs[-1] * 100, pedal_rpms[0], pedal_rpms[-1]],
            vmin=0.0,
            vmax=1.0,
            cmap="viridis",
        )
        mappable = image
        ax.set_title(f"Kv={kv:.0f}")
        ax.set_xlabel("Battery SOC (%)")

    axes[0].set_ylabel("Pedaling RPM")
    fig.suptitle(f"Best Gear Match Kv Impact | ratio={best['total_ratio']:.2f}")

    if mappable is not None:
        cb = fig.colorbar(mappable, ax=axes, fraction=0.028, pad=0.02)
        cb.set_label("Estimated regen efficiency (0..1)")

    plt.savefig(cfg.kv_comparison_plot_file, dpi=160)
    plt.close(fig)
    return cfg.kv_comparison_plot_file


def regen_threshold_summary(best: dict, cfg: SearchConfig, soc: float) -> list[str]:
    bus_v = lifepo4_cell_ocv_from_soc(soc) * cfg.battery_series_cells
    ratio = best["total_ratio"]
    rows: list[str] = []

    for kv in cfg.motor_kv_candidates:
        # EMF = pedal_rpm * ratio / kv. Threshold when EMF = bus_v.
        threshold_rpm = bus_v * kv / ratio
        rows.append(f"Kv={kv:.0f}: direct-regen threshold at ~{threshold_rpm:.1f} pedal RPM (SOC={soc*100:.0f}%)")

    return rows


def main() -> None:
    cfg = SearchConfig()
    results = search(cfg)

    print("=== Gearbox Search (efficient + low wear + backdrivable) ===")
    print(f"Gears: {list(cfg.gears)}")
    print(
        f"Stages: {cfg.min_stages}-{cfg.max_stages}, "
        f"Target total ratio: [{cfg.target_ratio_min}, {cfg.target_ratio_max}]"
    )
    print(f"Found {len(results)} valid configurations\n")

    if not results:
        print("No valid solution with current constraints.")
        print("Try one of:")
        print("- Increase max_stages")
        print("- Increase hard_stage_ratio_limit")
        print("- Widen target_ratio_min/target_ratio_max")
        return

    for index, item in enumerate(results[: cfg.top_n_results], start=1):
        stages = item["stages"]
        ratios = item["ratios"]
        stage_text = "; ".join(format_stage(stage, ratio) for stage, ratio in zip(stages, ratios))

        print(
            f"#{index:02d} | score={item['score']:.4f} | total_ratio={item['total_ratio']:.3f} | "
            f"eff={100 * item['efficiency']:.2f}% | backdrive={item['backdrive']:.3f} | "
            f"wear={item['wear']:.3f} | stages={len(stages)}"
        )
        print(f"     {stage_text}")

    best = results[0]
    plot_path = plot_efficiency_map(best, cfg, motor_kv=cfg.motor_kv_rpm_per_volt, file_name=cfg.plot_file)
    compare_path = plot_efficiency_map_kv_comparison(best, cfg)

    if plot_path:
        print(f"\nSaved SOC/RPM efficiency map for best match: {plot_path}")
    if compare_path:
        print(f"Saved Kv comparison SOC/RPM efficiency map: {compare_path}")

    for line in regen_threshold_summary(best, cfg, soc=0.5):
        print(line)


if __name__ == "__main__":
    main()
