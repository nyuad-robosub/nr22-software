This package uses the `object_detection` package provided in [TensorFlow API's repository](https://github.com/tensorflow/models/tree/master/research/object_detection). To install:
- Please clone the repository branch `r1.13.0`
- `cd` into `research`
- Do the following (courtesy of [rakidedigama](https://stackoverflow.com/a/57002353)):
```
python setup.py build
python setup.py install
```

If there is an issue with CUDA, possibly your graphics card is being used for display and cannot run TensorFlow (see [this](https://stackoverflow.com/questions/41965187/nvidia-device-error-in-tensorflow) and [this](https://stackoverflow.com/a/39661999)). Remove availability of CUDA devices could help (only show available devices not used for display):
```
export CUDA_VISIBLE_DEVICES=1
```

# Run these commands to install all dependencies:

```
pip2 install --user tensorflow==1.15.0
pip2 install --user Cython
pip2 install --user contextlib2
pip2 install --user pillow
pip2 install --user lxml
pip2 install --user jupyter
pip2 install --user matplotlib
```

Installing protobuf
If you are on linux:

Download and install the 3.0 release of protoc, then unzip the file.
```
# From tensorflow/models/research/(be in this directory)
wget -O protobuf.zip https://github.com/google/protobuf/releases/download/v3.0.0/protoc-3.0.0-linux-x86_64.zip
unzip protobuf.zip
rm -rf protobuf.zip
```

Run the compilation process again, but use the downloaded version of protoc

From tensorflow/models/research/
```
./bin/protoc object_detection/protos/*.proto --python_out=.
```