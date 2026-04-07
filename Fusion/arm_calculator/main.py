from fastapi import FastAPI
from fastapi.responses import Response, FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from calculator import calculate
from fusion_export import build_csv

app = FastAPI()


class ArmParams(BaseModel):
    # Geometry (mm)
    link1_length: float = 200
    link2_length: float = 180
    link3_length: float = 80
    tube_od: float = 22
    tube_wall: float = 2

    # Bearing / Hub (mm)
    shaft_dia: float = 8
    bearing_od: float = 22
    bearing_width: float = 7
    hub_od: float = 40

    # Mass (grams)
    motor_mass: float = 300
    hub_mass: float = 80
    gripper_mass: float = 400
    payload_mass: float = 500

    # Pulley / Gear
    spool_radius: float = 11
    elbow_pulley_radius: float = 15
    wrist_pulley_radius: float = 15
    shoulder_gear_ratio: float = 1.0
    elbow_gear_ratio: float = 1.0
    wrist_gear_ratio: float = 1.0

    # Pose (degrees)
    shoulder_angle: float = 45
    elbow_angle: float = 90
    wrist_angle: float = 0

    # Line
    line_break_strength: float = 133
    line_safety_factor: float = 3.0
    torque_safety_factor: float = 1.5

    # Motor target
    motor_torque_target: float = 1.5


@app.post("/calculate")
def calculate_endpoint(params: ArmParams):
    return calculate(params.model_dump())


@app.post("/export")
def export_endpoint(params: ArmParams):
    csv_data = build_csv(params.model_dump())
    return Response(
        content=csv_data,
        media_type="text/csv",
        headers={"Content-Disposition": "attachment; filename=arm_params_fusion360.csv"},
    )


app.mount("/", StaticFiles(directory="static", html=True), name="static")
