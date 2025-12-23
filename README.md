
# SELM-SLAM3

SELM-SLAM3 is a deep learning–enhanced RGB-D SLAM framework built upon **ORB-SLAM3**.
It integrates **SuperPoint** for feature extraction and **LightGlue** for feature matching using **ONNX Runtime (C++ API)**, enabling robust and efficient inference without Python or PyTorch at runtime.

SELM-SLAM3 is designed for challenging scenarios such as **low-texture environments** and **fast motion**.

## Key Contributions

* Replacement of ORB feature extraction with **ONNX-based SuperPoint**
* Replacement of descriptor matching with **ONNX-based LightGlue**
* New matching strategy tailored for deep features
* Fully C++ inference pipeline (no Python runtime dependency)

## Performance

Evaluations on **TUM RGB-D**, **ICL-NUIM**, and **TartanAir** datasets demonstrate:

* **87.84% average improvement** over ORB-SLAM3
* **36.77% improvement** over state-of-the-art RGB-D SLAM systems

This framework targets **real-world assistive navigation** applications, particularly for visually impaired users.

---

## Repository Origin

This repository was forked from
[SUPERSLAM3](https://github.com/isarlab-department-engineering/SUPERSLAM3),
which itself is based on **ORB-SLAM3**.

Unlike SuperSLAM3 (which relies on PyTorch), SELM-SLAM3 uses ONNX Runtime for all deep learning inference.

---

## System Requirements

* **OS:** Ubuntu 20.04 (tested)
* **Compiler:** GCC ≥ 8 (C++14 required)
* **CUDA:** 11.8
* **cuDNN:** compatible with CUDA 11.8
* **GPU:** NVIDIA GPU recommended
* **OpenCV:** 3.4.0 (required by ORB-SLAM3)

---

## Prerequisites

```bash
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update

sudo apt-get install build-essential cmake git pkg-config
sudo apt-get install libgtk2.0-dev libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python-dev python-numpy
sudo apt-get install libtbb2 libtbb-dev
sudo apt-get install libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libjasper-dev
sudo apt-get install libglew-dev libboost-all-dev libssl-dev
sudo apt-get install libeigen3-dev
sudo apt install libcanberra-gtk-module
```

---

## Pangolin (Visualization)

SELM-SLAM3 requires **Pangolin** for visualization.
Use the following **tested commit**:

```bash
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
git checkout 86eb4975fc4fc8b5d92148c2e370045ae9bf9f5d
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j16
sudo make install
```

---
## DBoW

**DBoW** inherited unchanged from [SUPERSLAM3](https://github.com/isarlab-department-engineering/SUPERSLAM3?tab=readme-ov-file#dbow3-dbow2-pangolin-and-g2o-included-in-thirdparty-folder).

---
## NVIDIA Driver, CUDA, and cuDNN

Install:

* NVIDIA Driver (compatible with CUDA 11.8)
* CUDA **11.8**
* cuDNN (matching CUDA 11.8)

Ensure `cuda_runtime.h` is available at:

```
/usr/local/cuda-11.8/targets/x86_64-linux/include
```

---

## LibTorch (C++ API)

```bash
wget -O LibTorch.zip https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcu118.zip
sudo unzip LibTorch.zip -d /usr/local
```

---

## OpenCV 3.4.0

The code has been tested with OpenCV 3.4.x.

---

## ONNX Runtime (C++ API)

### Download ONNX Runtime (GPU)

Download the **precompiled GPU version**:

```
https://github.com/microsoft/onnxruntime/releases/download/v1.16.1/onnxruntime-linux-x64-gpu-1.16.1.tgz
```

Extract and place it inside the `Thirdparty/` directory:

```
SELM-SLAM3/Thirdparty/onnxruntime-linux-x64-gpu-1.16.1/
```

---

## SuperPoint & LightGlue Models (ONNX)

### Download Models

Models were obtained from:

[https://github.com/AIDajiangtang/Superpoint-LightGlue-Image-Stiching](https://github.com/AIDajiangtang/Superpoint-LightGlue-Image-Stiching)

Merge the split zip files:

```bash
cat superpoint_lightglue.zip.00* > superpoint_lightglue.zip
unzip superpoint_lightglue.zip
```

### Model Paths

Place the ONNX models under:

```
SELM-SLAM3/Weights/BBPretrained_Models/
```

Required CMake options:

```bash
-DBBSUPERPOINT_WEIGHT_PATH="$SUPERSLAM3_HOME/Weights/BBPretrained_Models/superpoint.onnx"
-DBBLIGHTGLUE_WEIGHT_PATH="$SUPERSLAM3_HOME/Weights/BBPretrained_Models/superpoint_lightglue.onnx"
```

---

## Building SELM-SLAM3

```bash
cd ~
git clone git@github.com:banafshebamdad/SELM-SLAM3.git
cd SELM-SLAM3
```

Ensure CUDA headers are visible in `build.sh`:

```bash
-DCUDA_INCLUDE_PATH="/usr/local/cuda-11.8/targets/x86_64-linux/include"
```

Then build:

```bash
chmod +x build.sh
./build.sh
```

---

## Running SELM-SLAM3 on Public Datasets

SELM-SLAM3 follows the same execution interface as ORB-SLAM3 for RGB-D input.
The general command format is:

```bash
./Examples/RGB-D/rgbd_tum \
Vocabulary/ORBvoc.txt \
Examples/RGB-D/<CONFIG>.yaml \
<DATASET_PATH> \
<ASSOCIATIONS_FILE>
```

---

## TUM RGB-D Dataset (examle)

```bash
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM1.yaml \
/path/to/rgbd_dataset_freiburg1_xyz \
/path/to/rgbd_dataset_freiburg1_xyz/associations.txt
```

---

## TartanAir Dataset (examle)

```bash
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TartanAir.yaml \
/path/to/TartanAir/hospital_hard/P037 \
/path/to/TartanAir/hospital_hard/P037/associations.txt
```

---

## ICL-NUIM Dataset (examle)

```bash
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/ICL-NUIM.yaml \
/path/to/ICL-NUIM/living_room_traj1 \
/path/to/ICL-NUIM/living_room_traj1/associations.txt
```

---


## Dataset Conversion (TartanAir)

To convert TartanAir datasets into ORB-SLAM3 format:

🔗 [TartanAir to ORB-SLAM3 Converter](https://github.com/banafshebamdad/groundtruth_generation/tree/master/scripts/TartanAir/tartanair_convertor)

---

## Citation

If you use SELM-SLAM3 in your research, please cite:

```bibtex
@inproceedings{Bamdad2025SELM,
  title     = {Deep Learning-Powered Visual SLAM Aimed at Assisting Visually Impaired Navigation},
  author    = {Bamdad, M. and Hutter, H.-P. and Darvishy, A.},
  booktitle = {Proceedings of the 20th International Conference on Computer Vision Theory and Applications (VISAPP)},
  year      = {2025},
  publisher = {SCITEPRESS},
  url       = {https://www.scitepress.org/Papers/2025/133382/133382.pdf}
}
```

---

## Support

For **bugs, build issues, and usage questions**, please open a **GitHub Issue** in this repository.  
This helps keep discussions public and benefits other users as well.

## Contact

* **bamdad@ifi.uzh.ch**
* **banafshebamdad@gmail.com**


---

## Status

🚧 **Under development**
Documentation and evaluation scripts are being finalized.

*Last updated: December 12, 2025*

