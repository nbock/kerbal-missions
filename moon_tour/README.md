Author: Nolan Bock

Description: A path planning algorithm that finds a quick flight path between the moons of some body in Kerbal Space Program

See it in action:
https://youtu.be/p2r1pQEUlRg

A description of my planning strategy and why I chose it:
The strategy I used was to try to calculate several fast Hohmann-like burns from the
central body to each moon. Then, whichever was the fastest burn would be added to the
trip route and that burn's moon would be removed from the list of moons.
We repeat the process until we have a full route / empty list of moons. Once
the route is established, I do Hohmann-like calculations until I pick the
minimum of the Hohhmann-like options to get to the first moon. Then we make the transfer and glide to that moon. From
that moon's periapsis, we do a capture burn because escaping the moon would put
us on a very slow and long orbit that would elongate the trip. Once captured, we
move back towards the central body (Kerbin or Jool) and get an even lower orbit
around the central body. The motivation is again that lower orbits have a faster
period and are more flexible for fast burns to a moon since we might need to
wait until a certain orbiting angle and low orbits make this a very low cost maneuver. Additionally, Hohmann-like burns and escape back into lower orbits instead of burning into that orbit immediately both saves time and fuel. Now, we've completed one trip to a moon and start the process over for the next moon in the route until we have reached all moons.
