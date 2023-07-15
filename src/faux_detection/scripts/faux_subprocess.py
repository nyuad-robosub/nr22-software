import sys
import pickle
from PIL import Image
import pandas as pd
import struct
import io

import torch

def test(data, filename):
    with open(filename, 'w') as file:
        file.write(data)

def send_data(data, output=sys.stdout.buffer):
    """
    Send data to PIPE.buffer identified by output, only works reliably with bytes.
    """
    serialized_data = pickle.dumps(data, protocol=2)
    size = len(serialized_data)
    output.write(struct.pack('!I', size))
    output.write(serialized_data)
    output.flush()

def receive_data(input=sys.stdin.buffer):
    """
    Receive data to PIPE.buffer identified by input, only works reliably with bytes.
    """
    size_data = input.read(4)
    size = struct.unpack('!I', size_data)[0]
    data = input.read(size)
    return pickle.loads(data, encoding="bytes")

if __name__ == "__main__":
    # Get paths to model and weights
    yolov5_path = receive_data().decode()
    weights_path = receive_data().decode()

    # Load custom checkpoint courtesy of:
    # https://github.com/ultralytics/yolov5/discussions/5872#discussioncomment-2592644
    model = torch.hub.load(yolov5_path, 'custom', path=weights_path, source='local')
    model.conf = 0.02

    while True:
        img_bytes = receive_data()
        if img_bytes is None:  # No more images to process, exit the loop
            break
            
        image = Image.open(io.BytesIO(img_bytes))
        results = model(image)
        result_df = results.pandas().xyxy[0]
        send_data(result_df.to_json().encode())
