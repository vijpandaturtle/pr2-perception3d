## Project: Perception Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---
# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify).
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!

## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

[image1]: ./images/Confusion.png
[image2]: ./images/Confusion2.png
[image3]: ./images/world1.png
[image4]: ./images/world2.png
[image5]: ./images/world3(1).png

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

Exercise 1 mainly involved the use of python-pcl library to implement the following functions :
In this section, I used the following filters:
1. **Statistical Outlier Filter :** This filter reduces noise due to outliers in the data. This is commonly used for real world image data where noise is inevitable.

2. **Voxel Grid Downsampling :** This filter is used to downsample the number of points used to represent each point cloud (3D image).
Here I set the leaf size parameter to 0.01 .

3. **PassThrough Filter :** This filter is used to remove all the unnecessary parts of the image and to focus only on the table and objects.
For the purposes of this project, I created two passthrough filters, one for the objects on the table and the other for the boxes on either side of the table. This filter keeps all of the data within it's filter limits and discards the rest. For each of the passthrough filters I set a min_axis and max_axis limits and the axis (x/y/z)

4. **RANSAC Plane Segmentation :** The RANSAC segmentation method in this case, is used to separate the objects from the table. Here I use a max distance parameter of 0.01 along with the setting the model and method type and then extract the inliers and outliers separately.

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

For this section I used the DBSCAN algorithm, after carefully observing and experimenting with the k_means and DBSCAN algorithm. This algorithm is
a good option to use, when the number of classes/clusters in your data is unknown. For implementation of this section, I used the Euclidean Clustering functions available in the Point Cloud Library.
Some of the optimizations I made were,
1. Setting the max and min cluster size.
2. Setting the cluster tolerance.



#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
Here is an example of how to include an image in your writeup.


### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

The implementation of the pcl_callback function which was a combination of the code in exercise-1, 2 and 3 was done in parallel with the exercise lessons. That part of the code involved the use of various filters which were simple function calls, thanks to the python-pcl library.
So, the purpose of the project was to implement this perception pipeline in simulated environment with some added noise, to mimic the noise in data that we would obtain in a real-world environment. Moving on to the implementing the pipeline as a node, there were a few steps that needed to completed to make it work, all of which is consolidated into the pr2_mover function. First, I initialized all the ros message types and assigned corresponding values. This was done easily by looking up the messages types on the srv file and their definitions using the **ros message info** command.

There are five ROS messages in total that I needed to generate and pass five messages

```python
  test_scene_num = Int32() # Indicates the test world number
  object_name = String() # Name of the object from the pick_list.yaml
  arm_name = String() # 'Left' or 'Right' arm to be used based on object group
  pick_pose = Pose() # Location of object on the table
  place_pose = Pose() # Drop off point of object
```
After generating and assigning values to these messages I just had to publish these messages to the respective publishers, and also write the corresponding parameters for each object to the respective output.yaml file. There is an output.yaml for each test world.

]
#### Training and Inferring from the SVM

Generating the SVM was a tough job to debug. It kept repeating the message 'Invalid cloud detected' for each point cloud. However, training the model did produce train.sav and model.sav file containing the training data and the saved model respectively.
Below you can see the normalized and non-normalized confusion matrix of the SVM on the training data.

![alt text][image1]

However, this confusion matrix didn't seem to have uniform readings, some were really high while three blocks were relatively low. I wanted to improve this, so I increased the number of features captured by increasing the number of iterations in the loop in capture_features script. After that to improve the SVM accuracy, I also added the 'C' term with a value of 0.01 . This term is used to tell the SVM how much we want to misclassify a particular sample in the scene, so it has to be as small as possible. More information about SVM kernel tricks and optimization techniques can be found [here.](https://stats.stackexchange.com/questions/23614/parameters-to-change-for-different-kernels-for-svm)
After the above mentioned optimizations, this was my final confusion matrix.

![alt text][image2]

For the pick and place requests, I only generated the required parameters and wrote them to the .yaml file. Performing the actual pick and place
tasks would be a future endeavour.

#### Performing object recognition in the test worlds

##### Test World 1
![alt text][image3]

##### Test World 2
![alt text][image4]

##### Test World 3
![alt text][image5]
