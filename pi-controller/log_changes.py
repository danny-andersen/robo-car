import csv
import os
from copy import deepcopy
from datetime import datetime, timedelta

import config

# --- Your globals ---
# obstaclesCmd, obstacles, systemStatus, piStatus assumed defined above

# --- Internal snapshot of last-seen values ---
_last_snapshot = { "flat": None }

# --- Utility: flatten all values into a single ordered dict ---
def _flatten_state():
    
    flat = {}

    # systemStatus 
    for k, v in config.systemStatus.items():
        if k == "robotState": 
            # Convert numeric enum to text safely 
            if 0 <= v < len(config.ROBOT_STATE_NAMES):
                flat[f"systemStatus.{k}"] = config.ROBOT_STATE_NAMES[v]
            else:
                flat[f"systemStatus.{k}"] = f"UNKNOWN({v})"
        elif k == "proximitySensors": 
            flat[f"systemStatus.{k}"] = config.printableProximity(v)
        elif k == "errorField": 
            if 0 <= v < len(config.ERROR_FIELD_NAMES):
                flat[f"systemStatus.{k}"] = config.ERROR_FIELD_NAMES[v]
            else:
                flat[f"systemStatus.{k}"] = f"UNKNOWN({v})" 
        elif k == "timestamp": 
            abstime = config.lastBootTime + v/1000.0  # Timestamp is ms since boot
            dt = datetime.fromtimestamp(abstime)
            dt += timedelta(milliseconds=(v % 1000))   # Add back the ms part for better resolution in logs    
            flat[f"systemStatus.{k}"] = dt.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # Format with ms precision
        else:
            flat[f"systemStatus.{k}"] = v
        
    # piStatus
    for k, v in config.piStatus.items():
        if k == "lidarProximity": 
            flat[f"systemStatus.{k}"] = config.printableProximity(v)
        else:
            flat[f"piStatus.{k}"] = v

    # obstaclesCmd
    for k, v in config.obstaclesCmd.items():
        flat[f"obstaclesCmd.{k}"] = v

    # obstacles array
    for i, obs in enumerate(config.obstacles):
        for k, v in obs.items():
            flat[f"obstacles[{i}].{k}"] = v

    return flat


def _flatten_state_no_obstacles(): 
    """Used only for change detection.""" 
    flat = {} 
    # for k, v in config.obstaclesCmd.items():
    #     flat[f"obstaclesCmd.{k}"] = v

    # systemStatus 
    for k, v in config.systemStatus.items():
        if k == "robotState": 
            # Convert numeric enum to text safely 
            if 0 <= v < len(config.ROBOT_STATE_NAMES):
                flat[f"systemStatus.{k}"] = config.ROBOT_STATE_NAMES[v]
            else:
                flat[f"systemStatus.{k}"] = f"UNKNOWN({v})"
        elif k == "proximitySensors": 
            flat[f"systemStatus.{k}"] = config.printableProximity(v)
        elif k == "errorField": 
            if 0 <= v < len(config.ERROR_FIELD_NAMES):
                flat[f"systemStatus.{k}"] = config.ERROR_FIELD_NAMES[v]
            else:
                flat[f"systemStatus.{k}"] = f"UNKNOWN({v})" 
        elif k == "timestamp":
            #ignore timestamp for change detection
            flat[f"systemStatus.{k}"] = None
        else:
            flat[f"systemStatus.{k}"] = v
    for k, v in config.piStatus.items():
        if k == "lidarProximity": 
            flat[f"systemStatus.{k}"] = config.printableProximity(v)
        else:
            flat[f"piStatus.{k}"] = v
    return flat

# --- Utility: detect changes between snapshots ---
def _detect_changes(old, new):
    if old is None:
        return {k: True for k in new}  # first call: everything is "changed"
    return {k: (old[k] != new[k]) for k in new}


# --- Main function to call whenever globals may have changed ---
def record_status_change():
    global _last_snapshot

    full_flat = _flatten_state()
    # flat_no_obs = _flatten_state_no_obstacles()
    
    changed = _detect_changes(_last_snapshot["flat"], full_flat)
    # changed = full_flat
    
    if any(changed.values()):
        # Expand change flags to full_flat (obstacles always False) 
        full_changed = {}
        for key in full_flat: 
            if key in changed:
                full_changed[key] = changed[key]
            else: 
                full_changed[key] = False
                
        # --- Write CSV ---
        write_csv(full_flat)
        # --- Write HTML ---
        write_html(full_flat, full_changed)

    # Update snapshot
    _last_snapshot["flat"] = deepcopy(full_flat)

# --- CSV writer ---
def write_csv(flat):
    file_exists = os.path.exists("robo-status.csv")

    with open("robo-status.csv", "a", newline="") as f:
        writer = csv.writer(f)

        # Write header only once
        if not file_exists:
            writer.writerow(["timestamp"] + list(flat.keys()))

        writer.writerow([datetime.now().isoformat()] + list(flat.values()))


# --- HTML writer ---
def write_html(flat, changed):
    file_exists = os.path.exists("robo-status.html")

    with open("robo-status.html", "a") as f:
        if not file_exists:
            f.write("<html><body><table border='1'>\n")
            # f.write("<tr><th>Timestamp</th>")
            for key in flat.keys():
                f.write(f"<th>{key}</th>")
            f.write("</tr>\n")

        f.write("<tr>")
        # f.write(f"<td>{datetime.now().isoformat()}</td>")

        for key, value in flat.items():
            if changed[key]:
                f.write(f"<td style='background-color: yellow; font-weight: bold'>{value}</td>")
            else:
                f.write(f"<td>{value}</td>")
        f.write("</tr>\n")


# --- Optional: call this at program exit to close HTML table cleanly ---
def finalize_html():
    if os.path.exists("robo-status.html"):
        with open("robo-status.html", "a") as f:
            f.write("</table></body></html>")
