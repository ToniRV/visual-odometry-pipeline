<img src="resources/vo_pipeline.png" align="right" width="30%"/>
# README for VO pipeline

Monocular, visual odometry (VO) pipeline with the most essential features:

1. initialization of 3D landmarks
2. keypoint tracking between two frames
3. pose estimation using established 2D↔3D correspondences
4. triangulation of new landmarks.

## Results
Here is a quick overview of our results:
https://www.dropbox.com/sh/jvh6bk42ok4fu2e/AADP0zjf7AOGADLhL-1Z81gna?dl=0

## Installation
1. To install this source code, first clone the repo:
```
git clone git@github.com:ToniRV/visual-odometry-pipeline.git
```

2. Modify the paths to the datasets that you can download from [here](http://rpg.ifi.uzh.ch/docs/teaching/2016/kitti00.zip)
Copy this [file](visual-odometry-pipeline/src/Parameters/paths_example.txt) and rename it to paths.txt
Modify your paths inside this file accordingly.
Then store it in the Parameters folder.

3. Now you can head to matlab and open main. You can now press Run.

## Reference output
https://www.youtube.com/watch?v=MQ1iDi3S-TU&list=PLI5XgAFFHqZiAg6yd0XKOTRgkCE4QTm54&index=3#t=32.749926
