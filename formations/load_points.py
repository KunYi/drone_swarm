import json

def load_points(filename):
    with open(filename, 'r') as f:
        data = json.load(f)
    return data

def load_from_json(inputfile, offset_x=0, offset_y=0, offset_z=0):
    points = load_points(inputfile)["points"]
    return [(p["x"] + offset_x, p["y"] + offset_y, p["z"] + offset_z) for p in points]
