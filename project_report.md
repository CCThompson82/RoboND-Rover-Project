[//]: # (Image References)

[image1]: ./output/demo_images/warped_img.png
[image2]: ./output/demo_images/color_thresh.png
[image3]: ./output/demo_images/rocks_binary.png

# Project: Search and Sample Return

## Summary

The objective of this project was to implement a search and sample return
solution for a simulated Mars Rover expedition.  The simulated Rover was  fitted
with a digital camera sensor, and from this feed, simple computer vision
functions and control logic were utilized to direct the Rover to search a
simulated terrain, find interesting rock samples, and return them to the
starting location for the rover.  

## Demonstration of rubric requirements

### Training / Calibration

#### Navigable terrain filter

In order to determine rover headings that were navigable, a perspective
transform was implemented, followed by a basic computer vision color threshold
function to semantically label pixels as navigable or belonging to obstructions.

The perspective transforms uses an emperically determined ratio between pixel
locations taken from a forward facing camera sensor to the world frame positions
they represent looking down on the rover scene from above.  An example of a
perspective transform result can be seen in Figure 1.

![alt_text][image1]

**Figure 1** - Example of a perspective transform from the front facing camera
sensor to a world map.  The left panel shows an example warped view (looking down
above) of example image data.  The right panel shows the warped view of a
blank image, which illustrates the region of the world which can be sensed by
the front-facing camera sensor.  

To accomplish this transformation, the `cv2` module was utilized. First source
pixel locations from the sensor were linked to destination points in the world
frame.  Then functions `getPerspectiveTransform` and `warpPerspective` were
called in turn.  This logic is contained within the `perspect_transform`
function in the `perception` module. The results of these transformations are
shown in Figure 1, with the left panel representing the transformation of actual
sensor data, and the right panel indicating the regions of the world frame that
can possibly be reported given the perspective view of the front-facing camera
sensor.  

The warped transformations are then fed to a color threshold function.  A pixel
is deemed navigable if it passes a color filter for each of the red, green, and
blue digitial color channels.  A pixel is deemed as an obstruction if it fails
this threshold test AND is located within the available perspective of the
camera (e.g. within the yellow region of Figure 1, left panel).  

The result of this pipeline is illustrated in Figure 2.  The logic for this
determination is located in the `color_thresh` function in the `perception`
module.

![alt_text][image2]

**Figure 2**- Plot of navigable terrain afte perspective transform of an
on-rover camera sensor.  

#### Sample detection

Several interesting rock samples are located within the virtual Mars
environment, distinguished from other rocks by their gold color.  This
distinction was exploited  in order to locate interesting samples.  To do this,
a separate color threshold  function was created, called `find_rocks`, located
in the `perception` module. RGB threshold values of 100, 100, and 50 were
appeared suitable for the the distinction of the cold colored rocks from
obstacle and navigable terrain. An example of this filteration is shown in
Figure 3.  

![alt_text][image3]

**Figure 3** - Example of sample rock detection using color exploitation.  

### Strategy and Implementation

#### Perception

These steps were all made in a wrapping function, called `perception_step` in
the `perception` module.  By passing the current `Rover` object state to the
function, which includes the newest image from the camera sensor, a series
of functions are called that update the Rover object attributes.  

In brief, the basic process is as follows:
1. The incoming image is subjected to a perspctive transform, which converts
the image from the perspective of the Rover to a top-down map-like perspective.

2. The filtered map-like perspective image is then subjected to color
thresholding  functions, `color_thresh` and `rocks_binary`, in order to create
binary maps of  navigable (and thus non-navigable, see note below) terrain, as
well as for gold sample locations.  Notably, the obstacle map is filtered for
regions where the Rover  camera sensor can view, to avoid misclassifying regions
where no data has been  obtained.  

3. The resultant binary maps are in the frame of the Rover, and the Rover's
world position and heading (yaw) can be utilized to convert these rover-centric
map coordinates into world frame coordinates.  

4. Once converted into the world frame, a worldmap attribute is updated based on
the navigable, obstacled, and rock_sample coordinates.  Every 30 seconds, this
worldmap is cleaned in order to reduce infidelity per pixel.  

5. Finally, the world coordinates for this perception step are post-processed
into polar coordinates relative to the Rover.  These angles, distance lists
are stored as Rover attributes for use in the decision steps.  

#### Autonomous navigation and mapping

Having update Rover attributes within the perception phase, `decision_step`
function is called in order to determine actions taken by the Rover.  The
general strategy employed in this process was a loose attempt to keep the
right wall 7 meters from the rover.  For instance, if the wall was sensed to be
less then 7 meters from the rover, a left steer is applied, and if no obstacle
was detected to the right heading of the Rover, then a right steer would be
applied.  There are obvious limitations to this strategy, but this
implementation provided a non-random search of the Mars scene without
referencing the world or the addition of memory logic, which would have vastly
complicated the agent code base.  

In brief, the basic modes provided for rover selection were:

* forward - attempt to keep obstacle wall 7 meters due east of the Rover.  The
method for this was to check -35, -30, and -25 degrees for the average distance
to obstacles.  Projection of these vectors onto the 0 deg axis were calculated
and their difference from the target of 7 meters was used to adjust the Rover
steering angle.  

* stop - when the amount of navigable terrain directly in front of the rover
did not eclipse a lower threshold, the brakes are applied to avoid running
into an obstacle.  Once stopped, the rover steers towards the left until
navigable terrain is in view.  

* stuck - when the rover has not moved for a period of time, despite appearance
of navigable terrain ahead, a protocol for becoming unstuck is implemented.  The
steps for this protocol include steering, reversing and firing forward for
periods of time, irrespective of navigable terrain perception.

* standtrap - when the rover roll is outside a normal range, the rover steers
until it returns to level footing.  

* collection - when a sample is perceived, the rover enters into a collection
mode which is a protocol of steps that usually end up with the sample being
collected.  

* home - once all samples are collected, and the starting position from the
rover is near, the home mode is triggered which will stop the rover in the
same place the simulation began.  

This implementation generally works and can find, collect all samples, and
return to home position in about 20 minutes.  However, placement of rock samples
can have drastic effects on the software performance.  Attached to this
submission is a
[video](https://github.com/CCThompson82/RoboND-Rover-Project/blob/master/output/submission_run1.mp4.zip)
where the rover completed the mission successfully with resolution set to a high
quality and acheiving approximately 30 fps.    


### Issues and Potential Improvements

The logic surrounding the collection mode is very naive and can result in
unwanted consequences for the Rover.  Instead of storing where the sample is
located and attempting to navigate to that location, I employed a strategy of
attempting to keep the sample due straight ahead of the Rover.  Often the sample
will be sensed, but not relocated during the exploration spin of the rover.

Also, the logic that surrounds collection does not remember the original rover
heading.  After collection, the Rover can begin navigating towards a region it
had already explored, potentially doubling the exploration time.  To this end
no real memory of where the rover has explored is implemented in this code.  
Its inclusion could allow drastic reduction in mission completion time.  

One failure case that could arise with the wall crawling forward mode is that
if an environment contained no walls, the rover would not be able to explore
farther than the radius it steers over and over.  In other words, the navigation
logic is dependent on the presence of obstacles.  


### Summary

This project provides the software to complete a basic Mars Rover simulator
mission.  It utilizes perspective transform to convert camera sensor information
into a local map of the rover's heading environment.  Basic computer vision
techniques were used to assess navigable from obstacled terrain, as well as
detect interesting rock samples.  The results of this perception pipeline
informed a conditionals tree that controlled the behaviour of the rover.  The
final result was a basic implementation that could search a small Mars valley
environement, collect interesting rock samples, and return to a home location.  
