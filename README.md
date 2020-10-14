# DSM: Direct Sparse Mapping

**Authors:** Jon Zubizarreta, Iker Aguinaga, Juan D. Tardós and J. M. M. Montiel.

**Contact:** dsm (at) unizar (dot) es

DSM is a novel approach to monocular SLAM. It is a fully direct system that estimates the camera trajectory and a consistent global map. Is is able to detect and handle map point reobservations when revisiting already mapped areas using the same photometric model and map points. We provide examples to run the SLAM system in the [EuRoC dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) and with custom videos. We also provide an optional GUI for 3D visualization of the system results.

<a href="https://youtu.be/sj1GIF-7BYo" target="_blank"><img src="http://img.youtube.com/vi/sj1GIF-7BYo/0.jpg" 
alt="DSM" width="240" height="180" border="10" /></a>

## 1. Related Publications

* Jon Zubizarreta, Iker Aguinaga and J. M. M. Montiel. **Direct Sparse Mapping**. In *IEEE Transactions on Robotics* [[pdf](https://arxiv.org/abs/1904.06577)]

## 2. Installation
We tested DSM int two different system configurations: **Ubuntu 18.04** and **Windows 10** (VS15 and VS17). It should be easy to compile in other platforms. The library requires at least C++11.

Clone the repository:

```sh
git clone https://github.com/jzubizarreta/dsm.git
```

### 2.1 Required Dependencies

#### Eigen3

We use [Eigen3](http://eigen.tuxfamily.org) for almost any mathematical operation. 
	
Install with
	
```sh
sudo apt-get install libeigen3-dev
```

#### OpenCV

We use [OpenCV](https://opencv.org/) to manipulate images (read/write/display) and to bootstrap the monocular system. Feel free to implement those functionalities with your prefered library, if you want to get rid off Opencv. 
	
Install with
	
```sh
sudo apt-get install libopencv-dev
```

#### Ceres Solver

We use [Ceres Solver](http://ceres-solver.org) to perform the photometric bundle adjustment. In addition to the standard installation process, we generate a custom template specialization corresponding to our Schur structure, which speeds up the Schur elimination step. This is an optional step that can be skipped.
	
```sh
git clone https://ceres-solver.googlesource.com/ceres-solver
```
	
Ceres dependencies:
	
```sh
# glog & gflags
sudo apt-get install libgoogle-glog-dev

# BLAS & LAPACK
sudo apt-get install libatlas-base-dev

# SuiteSparse
sudo apt-get install libsuitesparse-dev
```
		
Generate the custom template specialization (optional):

```sh
cp ./dsm/thirdparty/Ceres/generate_template_specializations.py ./ceres-solver/internal/ceres/
python2 ceres-solver/internal/ceres/generate_template_specializations.py
```

Install with
	
```sh
cd ceres-solver
mkdir build
cd build
cmake ..
make -j4
sudo make install	
```

### 2.2 Optional Dependencies

#### Qt

We use [Qt](https://www.qt.io/) for GUI and visualization. Although Qt is required to compile the whole project in the current version, it is easy to remove it. DSM does not depend on Qt. All the code related with Qt is in the `QtVisualizer` folder. Feel free to implement your own version of `IVisualizer` and replace the current visualizer `QtVisualizer`.

Install with

```sh
sudo apt-get install qt5-default
```

### 2.3 DSM build

Now we are ready to build DSM. Execute:

```sh
cd ../../dsm
mkdir build
cd build
cmake ..
make -j4
```

This will compile two libraries `libdsm.so` and `libQtVisualizer.so` at **lib** folder, which can be linked from external projects. It will also create two executables `EurocExample` and `VideoExample` at **bin** folder to run DSM in the EuRoC dataset and with custom videos respectively.

## 3. Usage

### 3.1 Calibration format
DSM requires the geometric calibration of the camera as an input. Currently it uses the radial-tangential model (`camera_type` = `radtan`) or the fisheye model (aka Kannala-Brandt 4, aka pinhole-equidistant) (`camera_type` = `equidistant`) of OpenCV. However, it should be easy to add new camera models. Take a look at `Unidistorter.h`.

The calibration file has the format

```
camera_type
fx fy cx cy other parameters
in_width in_height
out_width out_height 
```

It is also possible to use higher order distortion models. You have to add the additional distortion coefficients using the OpenCV order. If no distortion coefficients are provided, they are assumed to be zero. Examples can be found in `Examples/EurocData` and `Examples/TUMVIData`.

### 3.2 EuRoC Example

1. Download a sequence (ASL format) from [https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets).

2. Extract the sequence in a folder `<SEQ_FOLDER>`.

3. Run the executable with the following arguments

```sh
./EurocExample <IMAGE_FOLDER> <TIMESTAMPS_FILE> <CALIB_FILE> <SETTINGS_FILE>
```

where 

   * `<IMAGE_FOLDER>` Folder containing the images of the sequence.

   * `<TIMESTAMPS_FILE>` File containing the image timestamps for that specific sequence.

   * `<CALIB_FILE>` File with the geometric calibration of the camera.

   * (Optional) `<SETTINGS_FILE>` File with the system settings. If it is not provided, internal default values are used.

The `<IMAGE_FOLDER>` can be usually found in `<SEQ_FOLDER>/mav0/camX/data`, where the X indicates if the left (0) or right (1) camera is used. The specific `<TIMESTAMPS_FILE>`, `<CALIB_FILE>` and `<SETTINGS_FILE>` for the EuRoC dataset are provided in `Examples/EurocData`.

The same executable can be used to run sequences from the TUM-VI dataset.

### 3.3 Video Example
It is also possible to run your own custom videos with known camera calibration. Run the executable as

```sh
./VideoExample <VIDEO_FILE> <CALIB_FILE> <SETTINGS_FILE>
```

where

   * `<VIDEO_FILE>` Custom video file with a supported format by OpenCV.

## 4. Parameter options
The system parameter options can be found in `settings.h`. Those can also be loaded using an external `.txt` file, such as the one in `Examples/EurocData/settings.txt`. The most relevant parameters are:

* `blockUntilMapped`: blocks the tracking thread until the mapping thread finishes.
* `singleThreaded`: runs tracking and mapping sequentially in a single thread.
* `mappingThreads`: number of multithreads to parallelize the mapping.
* `minimizeMemory`: use a buffer pool for memory usage minimization.
* `useFixedHuberScale`: flag to control if a Huber loss function with a fixed scale in used.
* `huberScale`: value of the Huber loss function scale.
* `useTDistribution`: flag to control if the t-distribution is used. When it is enabled, Huber loss is disabled.
* `nuFixed`: the t-distribution degrees of freedom. When <= 0 it is dinamically computed, otherwise it is fixed.
* `muFixed`: the t-distribution location. When < 0 it is dinamically computer, otherwise it is fixed.
* `sigmaFixed`: the t-distribution scale. When <= 0 it is dinamically computer, otherwise it is fixed.
* `defaultNu`: default value fot the t-distribution degrees of freedom.
* `defaultMu`: default value fot the t-distribution location.
* `defaultSigma`: default value fot the t-distribution scale.
* `inlierPercentile`: percentile of inliers for energy threshold.
* `maxPixelOutlier`: maximum percentage of pixels to consider an observation as outlier.
* `maxPixelDiscard`: maximum percentage of pixels to discard an observations during optimization.
* `maxEnergyFit`: maximum energy to fit the t-distribution.
* `weightConstant`: gradient weight constant.
* `trackingMaxLevel`: maximum number of pyramid levels during frame tracking.
* `pointDetectionLevels`: number of pyramid levels to detect candidate points.
* `numCandidates`: number of candidate points per keyframe.
* `numBlocksPerDimension`: number of blocks per image dimension during candidate point detection.
* `minGradAdd`: threshold constant addition to histogram for candidate point detection.
* `maxUnmappedFrames`: maximum number of unmmaped frames in the queue.
* `maxEplLengthFactor`: maximum epipolar search length.
* `minEplLengthSkip`: minimum epipolar search length.
* `stereoMaxEnergy`: maximum energy for epipolar search.
* `secondBestRadius`: radius to the second best match during epipolar search.
* `epiLineSigma`: epipolar line uncertainty.
* `subpixelIterations`: number of iterations during subpixel computation.
* `subpixelStepThreshold`: threshold to stop the subpixel optimization.
* `maxViewChange`: maximum parallax to consider a point as visible. 
* `candidateOptIterations`: maximum number of iterations for candidate refinement.
* `minDistToActivate`: initial minimum distance to activate new active points.
* `maxCandidateUncertainty`: maximum  candidate point uncertainty to activate it.
* `minCandidateQuality`: minimum candidate point quality (distance to second best) to activate it
* `doOnlyTemporalOpt`: use only temporally connected keyframe in the photometric bundle adjustment.
* `printSummary`: print bundle adjustment summary
* `showFullReport`: print bundle adjustment full report
* `minOptimizationGrad`: minimum gradient to consider an observation as outlier
* `minBAIterations`: minimum number of photometric bundle adjustment iterations.
* `maxBAIterations`: maximum number of photometric bundle adjustment iterations.
* `optMaxLevel`: number of maximum levels during pyramidal photometric bundle adjustment.
* `varScaleRot`: rotation scale factor during optimizations.
* `varScaleTrans`: translation scale factor during optimizations.
* `varScaleAlpha`: light alpha scale factor during optimizations.
* `varScaleBeta`: light beta scale factor during optimizations.
* `varScaleIDepth`: inverse depth scale factor during optimization.
* `numActivePoints`: maximum number of observations in the latest keyframe.
* `maxTemporalKeyframes`: maximum number of temporal keyframes in the optimization window.
* `maxCovisibleKeyframes`: maximum number of covisible keyframes in the optimization window.
* `numAlwaysKeepKeyframes`: number of latest temporal keyframes to keep always in the optimization window.
* `minPointCovisible`: minimum ratio of visible points to consider a keyframe as covisible.
* `maxLightCovisible`: maximum light change to consider a keyframe covisible.
* `minNumKFToConsiderNew`: minimum number of keyframes to consider an active point as a new.
* `minNumGoodObservations`: minimum number of observation to consider an active point as inlier.
* `newKFDistWeight`: weight of the camera translation relative to scene depth during new keyframe selection.
* `newKFUsageWeight`: weight of the point usage by the frame tracker during new keyframe selection.
* `newKFAffineWeight`: weight of the light change in the scene during new keyframe selection.
* `newKFResidualWeight`: weight of the frame tracker residual during new keyframe selection.
* `minNumMappedFramesToCreateKF`: minimum number of tracked frames to create a new keyframe.	

## 5. License

DSM is released under a [GPLv3 license](https://github.com/jzubizarreta/dsm/blob/master/License-gpl.txt). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/jzubizarreta/dsm/blob/master/Dependencies.md).

For a closed-source version of DSM for commercial purposes, please contact the authors.

If you use DSM in an academic work, please cite:

    @article{Zubizarreta2020,
      title={Direct Sparse Mapping},
      author={Zubizarreta, Jon, Aguinaga, Iker and Montiel, J. M. M.},
      journal={IEEE Transactions on Robotics},
      doi = {10.1109/TRO.2020.2991614},
      year={2020}
     }