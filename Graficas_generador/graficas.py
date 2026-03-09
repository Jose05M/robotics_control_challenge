import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


# ============================================================
# CONFIGURACIÓN
# ============================================================
CSV_PATH = "CTC_sin_perturbaciones.csv"     # cambia esto si tu archivo tiene otro nombre
OUTPUT_DIR = "resultados_ctc_sin_perturbaciones"

# Umbral para éxito de waypoint (metros)
WAYPOINT_THRESHOLD = 0.02   # 2 cm

# Número mínimo de muestras para considerar un waypoint/dwell
MIN_SEGMENT_SAMPLES = 5

# Cuántas muestras finales usar para el error promedio en dwell-window
DWELL_LAST_N = 10


# ============================================================
# FUNCIONES AUXILIARES
# ============================================================
def ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def rmse(a, b):
    a = np.asarray(a)
    b = np.asarray(b)
    return np.sqrt(np.mean((a - b) ** 2))


def detect_waypoint_segments(df, cols=("x_d", "y_d", "z_d"), tol=1e-12):
    """
    Detecta segmentos donde el waypoint deseado se mantiene constante.
    Regresa lista de tuplas: (start_idx, end_idx)
    """
    desired = df[list(cols)].values
    segments = []
    start = 0

    for i in range(1, len(df)):
        same = np.all(np.isclose(desired[i], desired[i - 1], atol=tol, rtol=0))
        if not same:
            if i - 1 - start + 1 >= MIN_SEGMENT_SAMPLES:
                segments.append((start, i - 1))
            start = i

    if len(df) - start >= MIN_SEGMENT_SAMPLES:
        segments.append((start, len(df) - 1))

    return segments


def compute_metrics(df):
    metrics = {}

    # --------------------------------------------------------
    # Errores articulares
    # --------------------------------------------------------
    joint_rmse = {}
    joint_max_error = {}

    for i in range(1, 7):
        q_col = f"q{i}"
        qref_col = f"qref{i}"
        if q_col in df.columns and qref_col in df.columns:
            err = df[q_col] - df[qref_col]
            joint_rmse[f"joint_{i}_rmse_rad"] = np.sqrt(np.mean(err**2))
            joint_max_error[f"joint_{i}_max_abs_error_rad"] = np.max(np.abs(err))

    metrics.update(joint_rmse)
    metrics.update(joint_max_error)

    # --------------------------------------------------------
    # Error de espacio cartesiano
    # --------------------------------------------------------
    if all(c in df.columns for c in ["x", "y", "z", "x_d", "y_d", "z_d"]):
        ex = df["x"] - df["x_d"]
        ey = df["y"] - df["y_d"]
        ez = df["z"] - df["z_d"]
        ee_err = np.sqrt(ex**2 + ey**2 + ez**2)

        metrics["ee_rmse_m"] = np.sqrt(np.mean(ee_err**2))
        metrics["ee_max_error_m"] = np.max(np.abs(ee_err))
        metrics["ee_mean_error_m"] = np.mean(ee_err)

    # --------------------------------------------------------
    # Dwell-window mean error
    # --------------------------------------------------------
    dwell_rows = []
    success_count = 0
    segments = detect_waypoint_segments(df)

    for k, (s, e) in enumerate(segments, start=1):
        seg = df.iloc[s:e+1].copy()

        ex = seg["x"] - seg["x_d"]
        ey = seg["y"] - seg["y_d"]
        ez = seg["z"] - seg["z_d"]
        ee_err = np.sqrt(ex**2 + ey**2 + ez**2)

        n_last = min(DWELL_LAST_N, len(seg))
        dwell_mean = ee_err.iloc[-n_last:].mean()
        final_err = ee_err.iloc[-1]

        success = final_err <= WAYPOINT_THRESHOLD
        success_count += int(success)

        dwell_rows.append({
            "waypoint_id": k,
            "start_index": s,
            "end_index": e,
            "start_time_s": seg["time"].iloc[0] if "time" in seg.columns else np.nan,
            "end_time_s": seg["time"].iloc[-1] if "time" in seg.columns else np.nan,
            "x_d": seg["x_d"].iloc[0],
            "y_d": seg["y_d"].iloc[0],
            "z_d": seg["z_d"].iloc[0],
            "dwell_mean_error_m": dwell_mean,
            "final_error_m": final_err,
            "success": success
        })

    dwell_df = pd.DataFrame(dwell_rows)

    if len(dwell_df) > 0:
        metrics["dwell_window_mean_error_m"] = dwell_df["dwell_mean_error_m"].mean()
        metrics["waypoint_success_rate_percent"] = 100.0 * dwell_df["success"].mean()
        metrics["num_waypoints"] = len(dwell_df)
    else:
        metrics["dwell_window_mean_error_m"] = np.nan
        metrics["waypoint_success_rate_percent"] = np.nan
        metrics["num_waypoints"] = 0

    return metrics, dwell_df


# ============================================================
# GRÁFICAS
# ============================================================
def plot_joint_tracking(df, output_dir):
    t = df["time"].values if "time" in df.columns else np.arange(len(df))

    fig, axs = plt.subplots(3, 2, figsize=(16, 12), sharex=True)
    axs = axs.ravel()

    for i in range(1, 7):
        ax = axs[i - 1]
        q_col = f"q{i}"
        qref_col = f"qref{i}"

        if q_col in df.columns:
            ax.plot(t, df[q_col], label=f"q{i}", linewidth=2)
        if qref_col in df.columns:
            ax.plot(t, df[qref_col], "--", label=f"qref{i}", linewidth=2)

        ax.set_title(f"Joint {i} Tracking")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Angle [rad]")
        ax.grid(True, alpha=0.3)
        ax.legend()

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "01_joint_tracking_all_6.png"), dpi=300, bbox_inches="tight")
    plt.close()


def plot_task_space_components(df, output_dir):
    t = df["time"].values if "time" in df.columns else np.arange(len(df))

    fig, axs = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    pairs = [("x", "x_d", "X"), ("y", "y_d", "Y"), ("z", "z_d", "Z")]

    for ax, (actual, desired, label) in zip(axs, pairs):
        if actual in df.columns:
            ax.plot(t, df[actual], label=f"{label} actual", linewidth=2)
        if desired in df.columns:
            ax.plot(t, df[desired], "--", label=f"{label} desired", linewidth=2)

        ax.set_ylabel(f"{label} [m]")
        ax.set_title(f"{label} Component")
        ax.grid(True, alpha=0.3)
        ax.legend()

    axs[-1].set_xlabel("Time [s]")
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "02_task_space_xyz.png"), dpi=300, bbox_inches="tight")
    plt.close()


def plot_3d_path(df, output_dir):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(df["x"], df["y"], df["z"], label="Actual path", linewidth=2)
    ax.plot(df["x_d"], df["y_d"], df["z_d"], "--", label="Desired path", linewidth=2)

    ax.scatter(df["x"].iloc[0], df["y"].iloc[0], df["z"].iloc[0], s=60, label="Start")
    ax.scatter(df["x"].iloc[-1], df["y"].iloc[-1], df["z"].iloc[-1], s=60, label="End")

    ax.set_title("3D End-Effector Path")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "03_3d_path.png"), dpi=300, bbox_inches="tight")
    plt.close()


def plot_ee_error_over_time(df, output_dir):
    t = df["time"].values if "time" in df.columns else np.arange(len(df))

    ex = df["x"] - df["x_d"]
    ey = df["y"] - df["y_d"]
    ez = df["z"] - df["z_d"]
    ee_err = np.sqrt(ex**2 + ey**2 + ez**2)

    plt.figure(figsize=(14, 5))
    plt.plot(t, ee_err, linewidth=2, label="||p - p_des||")
    plt.axhline(WAYPOINT_THRESHOLD, linestyle="--", linewidth=2,
                label=f"Success threshold = {WAYPOINT_THRESHOLD:.3f} m")
    plt.title("End-Effector Error Over Time")
    plt.xlabel("Time [s]")
    plt.ylabel("Position error norm [m]")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "04_ee_error_over_time.png"), dpi=300, bbox_inches="tight")
    plt.close()


def plot_phase_portraits(df, output_dir):
    fig, axs = plt.subplots(1, 3, figsize=(18, 5))
    
    for idx, i in enumerate([4, 5, 6]):
        q_col = f"q{i}"
        qd_col = f"qd{i}"

        axs[idx].plot(df[q_col], df[qd_col], linewidth=1.5)     
        axs[idx].set_title(f"Phase Portrait Joint {i}")
        axs[idx].set_xlabel(f"q{i} [rad]")
        axs[idx].set_ylabel(f"qdot{i} [rad/s]")
        axs[idx].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "05_phase_portraits_j1_j2_j3.png"), dpi=300, bbox_inches="tight")
    plt.close()


def plot_controller_outputs(df, output_dir):
    t = df["time"].values if "time" in df.columns else np.arange(len(df))
    fig, axs = plt.subplots(3, 2, figsize=(16, 12), sharex=True)
    axs = axs.ravel()

    for i in range(1, 7):
        ax = axs[i - 1]
        cmd_col = f"cmd{i}"
        if cmd_col in df.columns:
            ax.plot(t, df[cmd_col], linewidth=2, label=cmd_col)
        ax.set_title(f"Controller Output {cmd_col}")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Command")
        ax.grid(True, alpha=0.3)
        ax.legend()

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "06_controller_outputs.png"), dpi=300, bbox_inches="tight")
    plt.close()


def plot_perturbation_flag(df, output_dir):
    if "perturbation" not in df.columns:
        return

    t = df["time"].values if "time" in df.columns else np.arange(len(df))

    plt.figure(figsize=(14, 4))
    plt.plot(t, df["perturbation"], linewidth=2)
    plt.title("Perturbation Signal / Flag")
    plt.xlabel("Time [s]")
    plt.ylabel("Perturbation")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, "07_perturbation_flag.png"), dpi=300, bbox_inches="tight")
    plt.close()


# ============================================================
# TABLAS
# ============================================================
def save_metrics(metrics, dwell_df, output_dir):
    metrics_df = pd.DataFrame([metrics]).T.reset_index()
    metrics_df.columns = ["metric", "value"]
    metrics_df.to_csv(os.path.join(output_dir, "metrics_summary.csv"), index=False)

    if len(dwell_df) > 0:
        dwell_df.to_csv(os.path.join(output_dir, "dwell_waypoint_analysis.csv"), index=False)

    # Tabla comparativa simple
    comparison_rows = []

    for i in range(1, 7):
        rmse_key = f"joint_{i}_rmse_rad"
        max_key = f"joint_{i}_max_abs_error_rad"
        comparison_rows.append({
            "signal": f"Joint {i}",
            "RMSE": metrics.get(rmse_key, np.nan),
            "Max Error": metrics.get(max_key, np.nan),
            "Units": "rad"
        })

    comparison_rows.append({
        "signal": "End-effector norm",
        "RMSE": metrics.get("ee_rmse_m", np.nan),
        "Max Error": metrics.get("ee_max_error_m", np.nan),
        "Units": "m"
    })

    comparison_rows.append({
        "signal": "Dwell-window mean error",
        "RMSE": metrics.get("dwell_window_mean_error_m", np.nan),
        "Max Error": np.nan,
        "Units": "m"
    })

    comparison_rows.append({
        "signal": "Waypoint success rate",
        "RMSE": metrics.get("waypoint_success_rate_percent", np.nan),
        "Max Error": np.nan,
        "Units": "%"
    })

    comparison_df = pd.DataFrame(comparison_rows)
    comparison_df.to_csv(os.path.join(output_dir, "comparison_table.csv"), index=False)


# ============================================================
# MAIN
# ============================================================
def main():
    ensure_dir(OUTPUT_DIR)

    df = pd.read_csv(CSV_PATH)

    # Validación mínima
    required_cols = [
        "x", "y", "z", "x_d", "y_d", "z_d",
        "q1", "q2", "q3", "q4", "q5", "q6",
        "qd1", "qd2", "qd3",
        "qref1", "qref2", "qref3", "qref4", "qref5", "qref6",
        "cmd1", "cmd2", "cmd3", "cmd4", "cmd5", "cmd6"
    ]
    missing = [c for c in required_cols if c not in df.columns]
    if missing:
        raise ValueError(f"Faltan columnas necesarias en el CSV: {missing}")

    # Métricas
    metrics, dwell_df = compute_metrics(df)
    save_metrics(metrics, dwell_df, OUTPUT_DIR)

    # Gráficas requeridas
    plot_joint_tracking(df, OUTPUT_DIR)
    plot_task_space_components(df, OUTPUT_DIR)
    plot_3d_path(df, OUTPUT_DIR)
    plot_ee_error_over_time(df, OUTPUT_DIR)
    plot_phase_portraits(df, OUTPUT_DIR)

    # Extras útiles para rubric de logging
    plot_controller_outputs(df, OUTPUT_DIR)
    plot_perturbation_flag(df, OUTPUT_DIR)

    # Resumen en consola
    print("\n=== MÉTRICAS PRINCIPALES ===")
    for k, v in metrics.items():
        print(f"{k}: {v}")

    print(f"\nResultados guardados en: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
