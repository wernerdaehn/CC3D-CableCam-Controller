# Endpoints

Using the programming switch the CableCam can be brought into a mode where the endpoints are set. Driving the CableCam to the startpoint and tipping the end point switch makes the current position the start point limit. Then driving forward or reverse to the end point and tipping the end point switch a second time sets this as the second limit. 
When the programming switch on the TX is put back into the operational state, the CableCam can be driven only within those two points (if mode is endpoint with limiters).

To change the end points, the programming mode is entered again and the CableCam allows driving to any position, even outside of the previous set range. The sequence is always the same, first click after entering the programming mode sets the first point, all subsequent clicks modify the second end point.

[![Using the CableCam Controller](../_images/Using_the_CableCam_Controller_YouTube_Video.jpg)](https://youtu.be/ohzvkKzsO8Q "Using the CableCam Controller")



The logic for the end point limiter is a bit tricky. 
Based on the stick position and the acceleration limit, the time to zero is calculated. For example, if the stick is at position +100% and the acceleration is 0.2, it takes 5 seconds to stop (=1.00 * 1/0.2). At a stick position of 50% half the time (=0.5 * 1/0.2). 

From the position sensor the controller knows the current speed, the time to stop from the stick level, hence the distance to stop can be calculated: s = v * t / 2 [(velocity-time graph)](http://www.bbc.co.uk/education/guides/z3bqtfr/revision/5). Example: s = 200steps/s * 10s / 2 = 1000steps. (One step is one increment of the position sensor)

This calculation is made in absolute numbers and the endpoint to be considered depends on the stick direction. Moving away from an endpoint is okay, it should limit moving into an endpoint.

If the brake distance in relationship to the current positions shows that the end point will be overshot, the controller starts to reduce the speed output.

There are two failsafes implemented in addition:

1. If the calculation shows the end point will be overshot by more than \$g steps, then it does engage an emergency brake. It outputs a speed signal of neutral. This causes the ESC to brake as hard as it can.
2. Once the end point is overshot the output signal is put into neutral as well to force a stop as quickly as possible.

Both failsafes have positive and negative effects.

In a perfect world, the speed signal is reduced linear and the CableCam stops exactly at the end point. In reality however the inertia, latency and how good the ESC can be controlled put physical limits.

Above works well if the ESC supports Closed Loop operations and the acceleration is set to a value the ESC can handle easily. The speed signal is constantly lowered, a fraction of a second later the ESC does achieve the desired speed and the end point will be overshot with very little excess speed and stop. Little excess speed hopefully, else it stops abrupt at this point.

To allow this little excess speed to be reduced gracefully there is an offset (\$o) values. The internal calculation tries to stop at the endpoints but moved inward by the offset. And only once the actual endpoint is overshot, the controller puts the speed to zero for an immediate stop.

On the other hand, if the acceleration setting is "stop from 100km/h to zero in one second", this will likely not work. That would be a deceleration of 2g. Therefore it is important to set the values to something that is realistic. Given the capabilities of the ESCs, the accelerations and decelerations are much higher than even sports cars can achieve. A deceleration of 1g is no problem. But this does not provide nice and smooth shots.

On the negative side, as soon as the emergency brake kicks is, there is a significant change in brake force. For closed loop ESCs the emergency brake should never engage, hence increasing \$g from 100 to something higher might make sense. For all other ESCs the emergency brake is part of the normal brake process and a value of 100 worked out well.

## Associated commands

| Command           | Allowed values | Description                                                  |
| ----------------- | -------------- | ------------------------------------------------------------ |
| r               |                | Print the ESC motor direction                                |
| r \<direction\> | 1, 0, -1       | Set the ESC motor direction. Does the pos sensor report larger values when the RC speed signal increase? Then +1. |
| g               |                | Print the max allowed positional error before the emergency brake kicks in |
| g \<steps\>     | \>0            | Set the max allowed position error                           |
| \$o |  | Print the end point offset |
| \$o <steps> | 0...1000 | Set the end point offset. This is the mathematical point the controller will try to stop at. The cablecam's inertia will cause to overshoot that point slightly. |



## Setup

For the endpoints to work properly, the \$r setting is important. Once an endpoint has been overshot, moving the CableCam back into the safe area is allowed, driving deeper into the forbidden area is not. Therefore it is important for the controller to know what the receiver input means for the ESC, driving into the one or the other direction.

To simplify the setup, the default is \$r 0 meaning the controller should watch the cablecam during the end point setting and set \$r itself. It does that by calculating the integral of all speed signals starting with the first position. If the end position is higher than the start position and the ESC signal was positive, then \$r 1 is set. A stick >0% means increasing the position. Hence the controller knows that at the start point a stick >0% drives the CableCam back into the safe zone, at the end point a value <0% drives it back into the safe zone.

Might be a good idea to save that setting afterwards.