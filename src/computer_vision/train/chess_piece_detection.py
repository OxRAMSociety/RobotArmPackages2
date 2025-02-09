# Temporary files for loading the model
from roboflow import Roboflow
import yaml
from ultralytics import YOLO

try:
    with open(".roboflow_key", "r") as f:
        ROBOFLOW_KEY = f.read().strip()
except FileNotFoundError:
    from sys import exit, stderr

    print("Please create a `.roboflow_key` file with the API key", file=stderr)
    exit(1)

# Export dataset
rf = Roboflow(api_key=ROBOFLOW_KEY)
project = rf.workspace("oxram").project("chess_pieces-e6ln1")
# Download last version in a specific format
dataset = project.versions()[0].download(model_format="yolov8-obb", location="data")

# Allows us to open the train dataset
with open(f"{dataset.location}/data.yaml", "r") as file:
    data = yaml.safe_load(file)

data["path"] = dataset.location

with open(f"{dataset.location}/data.yaml", "w") as file:
    yaml.dump(data, file, sort_keys=False)

model = YOLO("yolov8n.pt")

results = model.train(data=f"{dataset.location}/data.yaml", epochs=100, imgsz=640)
