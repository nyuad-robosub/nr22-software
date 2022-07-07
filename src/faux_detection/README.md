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