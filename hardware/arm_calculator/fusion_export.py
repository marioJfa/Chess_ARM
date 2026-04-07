import csv
import io


def build_csv(params: dict) -> str:
    tube_od = params["tube_od"]
    tube_wall = params["tube_wall"]
    tube_id = tube_od - 2 * tube_wall
    spool_dia = params["spool_radius"] * 2
    pulley_radius = params["elbow_pulley_radius"]

    rows = [
        # Name, Unit, Expression, Value, Comment, Favorite
        ("link1_length",           "mm",  str(params["link1_length"]),   params["link1_length"],   "Upper arm length",          "false"),
        ("link2_length",           "mm",  str(params["link2_length"]),   params["link2_length"],   "Forearm length",            "false"),
        ("link3_length",           "mm",  str(params["link3_length"]),   params["link3_length"],   "Wrist link length",         "false"),
        ("tube_od",                "mm",  str(tube_od),                  tube_od,                  "Tube outer diameter",       "false"),
        ("tube_wall",              "mm",  str(tube_wall),                tube_wall,                "Tube wall thickness",       "false"),
        ("tube_id",                "mm",  str(round(tube_id, 2)),        round(tube_id, 2),        "Tube inner diameter",       "false"),
        ("shaft_dia",              "mm",  str(params["shaft_dia"]),      params["shaft_dia"],      "Drive shaft diameter",      "false"),
        ("bearing_od",             "mm",  str(params["bearing_od"]),     params["bearing_od"],     "Bearing outer diameter",    "false"),
        ("bearing_width",          "mm",  str(params["bearing_width"]),  params["bearing_width"],  "Bearing width",             "false"),
        ("hub_od",                 "mm",  str(params["hub_od"]),         params["hub_od"],         "Hub outer diameter",        "false"),
        ("spool_dia",              "mm",  str(spool_dia),                spool_dia,                "Cable spool diameter",      "false"),
        ("pulley_radius",          "mm",  str(pulley_radius),            pulley_radius,            "Elbow/wrist pulley radius", "false"),
        ("connector_clearance",    "mm",  "0.3",                         0.3,                      "Print fit clearance",       "false"),
        ("connector_socket_depth", "mm",  "25",                          25,                       "Socket engagement depth",   "false"),
        ("bolt_circle_dia",        "mm",  "30",                          30,                       "Bolt circle diameter",      "false"),
        ("base_limit_deg",         "deg", "340",                         340,                      "Base yaw travel",           "false"),
        ("shoulder_limit_neg_deg", "deg", "-10",                         -10,                      "Shoulder lower limit",      "false"),
        ("shoulder_limit_pos_deg", "deg", "150",                         150,                      "Shoulder upper limit",      "false"),
        ("elbow_limit_neg_deg",    "deg", "0",                           0,                        "Elbow lower limit",         "false"),
        ("elbow_limit_pos_deg",    "deg", "150",                         150,                      "Elbow upper limit",         "false"),
        ("wrist_limit_neg_deg",    "deg", "-90",                         -90,                      "Wrist lower limit",         "false"),
        ("wrist_limit_pos_deg",    "deg", "90",                          90,                       "Wrist upper limit",         "false"),
    ]

    buf = io.StringIO()
    writer = csv.writer(buf)
    writer.writerow(["Name", "Unit", "Expression", "Value", "Comment", "Favorite"])
    for row in rows:
        writer.writerow(row)
    return buf.getvalue()
