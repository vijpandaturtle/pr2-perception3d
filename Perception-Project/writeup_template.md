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

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

Exercise 1 mainly involved the use of python-pcl library to implement the following functions :
1. **Voxel Grid Downsampling**
   Voxel stands for a volume element in a 3D image (point cloud). This function reduces the number of points in a point cloud, while retaining
   the point cloud data.
2. **Pass Through Filtering**
   Removes the useless data from the point cloud, so the point cloud consists of the region of interest (in this case the robot's reachable workspace).
3. **RANSAC plane fitting** (This was followed by and extract indices step to extract the inliers and outliers in the image)
   This is used to further filter out the objects in the scene which are unnecessary i.e outliers.
The exercise-1 code can be found in the perception.py script under the respective comment.

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

Exercise 2 involved the use of Euclidean clustering algorithm also known as DBSCAN instead of the k-means. We come to this conclusion because DBSCAN is a better choice when the number of clusters are unknown.

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.
Here is an example of how to include an image in your writeup.

![demo-1](https://user-images.githubusercontent.com/20687560/28748231-46b5b912-7467-11e7-8778-3095172b7b19.png)

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

The implementation of the pcl_callback function which was a combination of the code in exercise-1, 2 and 3 was done in parallel with the exercise lessons. That part of the code involved the use of various filters which were simple function calls, thanks to the python-pcl library.
So, the purpose of the project was to implement this perception pipeline in simulated environment with some added noise, to mimic the noise in data that we would obtain in a real-world environment. This required the use of a statistical outlier filter to filter the extra noise out of each frame.
Now, moving on to the implementing the pipeline as a node. There were a few steps that needed to completed to make it work, all of which is consolidated into the pr2_mover function. First, I initialized all the ros message types and assigned corresponding values. This was done easily by looking up the messages types on the srv file and their definitions using the **ros message info** command.

##### Training and Inferring from the SVM

Generating the SVM was a tough job to debug. It kept repeating the message 'Invalid cloud detected' for each point cloud. However, training the model did produce train.sav and model.sav file containing the training data and the saved model respectively.
Below you can see the normalized and non-normalized confusion matrix of the SVM on the training data.
![image-1](..\images\Confusion.jpg) 
