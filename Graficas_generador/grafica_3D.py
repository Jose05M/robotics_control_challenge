import numpy as np
import pandas as pd


CSV_PATH = "PD_sin_perturbaciones.csv"
OUTPUT_CSV = "dwell_window_analysis.csv"

# Solo umbral de llegada
ARRIVAL_THRESHOLD = 0.02   # metros


def detect_waypoint_segments(df, cols=("x_d", "y_d", "z_d"), tol=1e-12):
    """
    Detecta segmentos donde el waypoint deseado permanece constante.
    Regresa lista de tuplas: (start_idx, end_idx, waypoint_xyz)
    """
    desired = df[list(cols)].values
    segments = []
    start = 0

    for i in range(1, len(df)):
        same = np.all(np.isclose(desired[i], desired[i - 1], atol=tol, rtol=0))
        if not same:
            wp = tuple(desired[start])
            segments.append((start, i - 1, wp))
            start = i

    wp = tuple(desired[start])
    segments.append((start, len(df) - 1, wp))
    return segments


def main():
    df = pd.read_csv(CSV_PATH)

    required = ["time", "x", "y", "z", "x_d", "y_d", "z_d"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"Faltan columnas en el CSV: {missing}")

    # Error cartesiano del efector final
    df["ee_error"] = np.sqrt(
        (df["x"] - df["x_d"]) ** 2 +
        (df["y"] - df["y_d"]) ** 2 +
        (df["z"] - df["z_d"]) ** 2
    )

    segments = detect_waypoint_segments(df)

    results = []

    for idx, (start, end, wp) in enumerate(segments, start=1):
        seg = df.iloc[start:end + 1].copy().reset_index(drop=False)
        seg = seg.rename(columns={"index": "global_index"})

        # Primer instante en que entra al umbral
        inside = seg["ee_error"] <= ARRIVAL_THRESHOLD

        if not inside.any():
            results.append({
                "waypoint_id": idx,
                "x_d": wp[0],
                "y_d": wp[1],
                "z_d": wp[2],
                "arrival_idx": np.nan,
                "arrival_time": np.nan,
                "leave_idx": np.nan,
                "leave_time": np.nan,
                "dwell_duration_s": 0.0,
                "dwell_mean_error_m": np.nan,
                "dwell_max_error_m": np.nan,
                "reached_waypoint": False,
                "met_1s_requirement": False
            })
            continue

        arrival_local = np.argmax(inside.values)   # primer True
        arrival_global_idx = int(seg.loc[arrival_local, "global_index"])
        arrival_time = float(seg.loc[arrival_local, "time"])

        # El "leave" es el final del segmento actual
        leave_global_idx = int(seg.iloc[-1]["global_index"])
        leave_time = float(seg.iloc[-1]["time"])

        dwell_seg = seg.iloc[arrival_local:].copy()

        dwell_duration = leave_time - arrival_time
        dwell_mean_error = dwell_seg["ee_error"].mean()
        dwell_max_error = dwell_seg["ee_error"].max()

        results.append({
            "waypoint_id": idx,
            "x_d": wp[0],
            "y_d": wp[1],
            "z_d": wp[2],
            "arrival_idx": arrival_global_idx,
            "arrival_time": arrival_time,
            "leave_idx": leave_global_idx,
            "leave_time": leave_time,
            "dwell_duration_s": dwell_duration,
            "dwell_mean_error_m": dwell_mean_error,
            "dwell_max_error_m": dwell_max_error,
            "reached_waypoint": True,
            "met_1s_requirement": dwell_duration >= 1.0
        })

    results_df = pd.DataFrame(results)

    # Dwell-window mean error global:
    # promedio de los dwell_mean_error de los waypoints alcanzados
    reached_df = results_df[results_df["reached_waypoint"] == True]
    if len(reached_df) > 0:
        overall_dwell_mean_error = reached_df["dwell_mean_error_m"].mean()
    else:
        overall_dwell_mean_error = np.nan

    success_1s_rate = 100 * results_df["met_1s_requirement"].mean()

    print("====================================================")
    print("DWELL-WINDOW ANALYSIS")
    print("====================================================")
    print(f"Arrival threshold: {ARRIVAL_THRESHOLD:.3f} m")
    print(f"Waypoints detectados: {len(results_df)}")
    print(f"Waypoints alcanzados: {results_df['reached_waypoint'].sum()}")
    print(f"Waypoints con dwell >= 1 s: {results_df['met_1s_requirement'].sum()}")
    print(f"Tasa de cumplimiento de 1 s: {success_1s_rate:.2f}%")
    print(f"Dwell-window mean error global: {overall_dwell_mean_error:.6f} m")
    print()

    print(results_df[[
        "waypoint_id", "x_d", "y_d", "z_d",
        "arrival_time", "leave_time",
        "dwell_duration_s", "dwell_mean_error_m",
        "reached_waypoint", "met_1s_requirement"
    ]].to_string(index=False))

    results_df.to_csv(OUTPUT_CSV, index=False)
    print(f"\nTabla guardada en: {OUTPUT_CSV}")


if __name__ == "__main__":
    main()
