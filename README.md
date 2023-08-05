# webxr_ros_pub

Experiment publishing webxr pose to ROS via rosbridge

Have taken webxr example [ar-barebones.html](https://github.com/immersive-web/webxr-samples/blob/main/ar-barebones.html), and added a rosbridge publisher based on [roslibjs tutorials](http://wiki.ros.org/roslibjs/Tutorials/BasicRosFunctionality)

## Notes

Linear and angular vel apparently not available in the pose:
https://github.com/immersive-web/webxr/issues/185

Wanted to measure latency when usb tethering, but
 * currently using port forwarding to run on phone at localhost url
 * without localhost, ar does not run (insecure context)
 * port forwarding requires usb debugging (?)
 * cannot usb tether and port forward 
 * tried copy file to phone - did not run (I guess web server is required)
 * tried running python server in tmux - couldnt install python
Longer term I guess I need to 
 * package as app?
 OR
 * serve with certs. 
Would be good to measure if web page version increases latency

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

## Usage:

ROS server side:
`ros2 run rosbridge_server rosbridge_websocket`

For android:

On laptop, go to
chrome://inspect/#devices

enable port forwarding

run web server

## TODO: 
* UI to specify webserver location
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
