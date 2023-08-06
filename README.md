# webxr_ros_pub

Quick experiment using rosbridge to publish webxr device pose to ROS.

Have taken webxr example [ar-barebones.html](https://github.com/immersive-web/webxr-samples/blob/main/ar-barebones.html), and added a rosbridge publisher based on [roslibjs tutorials](http://wiki.ros.org/roslibjs/Tutorials/BasicRosFunctionality)

![Quick test walking around home](./quick_walk_test_home.png)

**Figure**: Trajectory from running the webxr publisher, quickly walking around my home, logging `/pose` then plotting with [`evo`](https://github.com/MichaelGrupp/evo). Trajectory covered two rooms upstairs, staircase descent, walking through two rooms downstairs, then returning to the starting pose upstairs. Trajectory looks decent, no significant drift observed from start to end pose.


## Usage:

On the phone, I've got it running by running a webserver on the phone (see notes on distribution below). Roughly here's what I did:

First, on a linux laptop with ROS install (in my case, humble):
* Install rosbridge server: `sudo apt install ros-humble-rosbridge-server`
* Run the server:
    ```bash
    . /opt/ros/humble/setup.bash
    ros2 run rosbridge_server rosbridge_websocket
    ```

Then prepare the phone (in my case, an android device)
* Install [Simple HTTP Server](https://play.google.com/store/apps/details?id=com.phlox.simpleserver)
* Set it up to serve directory `Downloads/webxr_ros` on port 8080, start the server
* from this repo, download `webxr_ros_pub.html` into that directory
* in the phone browser, navigate to `localhost:8080/webxr_ros_pub.html`
  The page should look like the following:
  ![page screenshot](./screenshot.png)
* Update the websocket url with the ip of the linux laptop, click connect. You should see a popup confirming a successful connection
* Click "Enter AR" to start the webxr session.

You should now see the device pose displayed on screen, and the pose should also be publishing to ROS via the bridge server. Confirm with:
```
ros2 topic echo /pose
```

## debugging

On laptop, go to
chrome://inspect/#devices

enable usb debugging on the phone, connect phone to laptop

open page on phone, then inspect from laptop

## Misc Notes

Linear and angular vel apparently not available in the pose:
https://github.com/immersive-web/webxr/issues/185

Distribution:
* Ideally the html page could be hosted, say, on github pages, and the user could browse to that page, and enter the settings to connect to their rosbridge server. BUT:
* webxr wants secure origin, which means either serving the html on localhost on the phone (requires webserver), or serving externally via https, which then requires the websocket to use ssl as well (mixed content issue). So user either needs to setup a webserver on the phone, or setup certs for the websocket. I'm still wondering if packaging into a native app is an option, but it sounds like android webviews do not support webxr right now, at this point it probably makes sense to drop webxr and use unity or native ARCore api.

Latency: haven't measured, but from visual RViz inspection I'd guess in the 0.1s to 0.2s range. Would be good to measure somehow. I'd say this latency might be acceptable for general localisation applications, but may be problematic if the intent is to use this output for odometry.

Got pose out, 30Hz
 - check if faster rate possible?
 - look into transform from webxr frame to REP compliant? (e.g. z up) Just publish static tf to webxr frame

Attempting hosting on github pages, but now getting errors RE insecure websockets on pages loaded via https
```
roslib.js:3466 Mixed Content: The page at 'https://tim-fan.github.io/webxr_ros_pub/webxr_ros_pub.html' was loaded over HTTPS, but attempted to connect to the insecure WebSocket endpoint 'ws://192.168.1.250:9090/'. This request has been blocked; this endpoint must be available over WSS.
Ros.connect @ roslib.js:3466
webxr_ros_pub.html:1 Uncaught DOMException: Failed to construct 'WebSocket': An insecure WebSocket connection may not be initiated from a page loaded over HTTPS.
    at Ros.connect (https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.js:3466:18)
    at new Ros (https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.js:3444:10)
    at https://tim-fan.github.io/webxr_ros_pub/webxr_ros_pub.html:63:15
```

## TODO: 
* try different reference space (unbounded? will need to request feature in requestSession)
* look into depth api
* consider calc velocity via differentiation? Better try access IMU? 
* log some trajectories and inspect
 * can use for teach and repeat? 
    * Does trajectory drift over time? Does this change with different reference spaces? Can anchors be used to mitigate drift?
* look into publishing anchors
* best way to distribute?
 * could host on webserver, but webxr requires https, so rosbridge will require secure websocket, so users will have to setup/install certs to use? 
 * alternative - try bundle html into app, perhaps webxr will be happy and websockets can still be insecure
