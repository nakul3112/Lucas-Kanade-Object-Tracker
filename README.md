# Lucas-Kanade-Object-Tracker
Implementation of Lucas-Kanade template tracker for tracking objects.

One incredibly important aspect of human and animal vision is the ability to follow objects and
people in our view. Whether it is a tiger chasing its prey, or you trying to catch a basketball,
tracking is so integral to our everyday lives that we forget how much we rely on it.

To initialize the tracker, we need to define a template by drawing a bounding box around the object
to be tracked in the first frame of the video. For each of the subsequent frames the tracker will
update an affine transform that warps the current frame so that the template in the first frame is
aligned with the warped current frame.

The repository contains the python codes for tracking various entities in the image sequence.

car.py - For tracking car
human.py - For tracking human
vase.py - For tracking vase

Refer to the project report LK.pdf for further detials regarding the algorithm of Lucas-Kanade tracker. 

Following is the google drive link for the output videos.

https://drive.google.com/drive/folders/1HxfjfK__EUG9v9Yo_lXv5j2JV2Q1dNJ5?usp=sharing
