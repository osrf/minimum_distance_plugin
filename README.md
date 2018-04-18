# minimum_distance_plugin

This is a Gazebo "world plugin" for calculating the minimum distance between convex shapes. The convex shapes are attached to links within the simulation.

:warning: The `distance` field in the `gazebo::msgs::Contact` that gets published by this plugin refers to **penetration** distance. For bodies that are separated, this value will be negative. Therefore, this plugin is actually reporting the *maximum penetration* distance (which is the negative of the *minimum separation* distance).

The currently supported convex shapes include:

* Box
* Cylinder
* Sphere
* Capsule

The boxes and cylinders can be dilated, which turns them into qualitatively different shapes (their edges get rounded out).

To use the plugin, add the following element inside the `<world>` element of your `.world` file:

```
<plugin name="minimum_distance_plugin" filename="libminimum_distance_plugin.so">
  <!-- Keep reading to find out what to put here -->
</plugin>
```

You can look at the file `worlds/test.world` for an example of how to fill in the `<plugin>` tag for this plugin. Here's a breakdown of the available tags:

* `<pair>` designates a pair of convex object groups to compare when calculating the minimum distance. You can have arbitrarily many `<pair>` tags within your `<plugin>` element.
   * `name="..."` is an optional attribute for `<pair>`. Replace `...` to give your pair a specific name. The results of the test will be published to the `/default/minimum_distance_plugin/...` topic where `default` is the world's name and `...` is the string that you give to `<pair name="...">`. By default, the pair name will be `Test#` where `#` is replaced by how many `<pair>` tags preceded the current one. So the first `<pair>` is automatically given the name `Test0`, the second is named `Test1`, etc.

* `<from>` and `<to>` go inside of `<pair>`. Each `<pair>` must have exactly one `<from>` tag and exactly one `<to>` tag. The plugin will report the shortest distance between the group of geometries in the `<from>` tag and the group of geometries in the `<to>` tag.

* `<model name="...">` goes inside of the `<from>` or `<to>` tag. `...` should refer to a model name that is defined within the top-level `<world>` tag. You can add arbitrarily many `<model name="...">` tags to the `<from>` and `<to>` tags.

* `<link name="...">` goes inside of the `<model>` tag. Similarly to `<model>`, the `...` should refer to a link name that is defined within the model being referred to by the parent `<model>` tag. You can have arbitrarily many `<link name="...">` tags within each `<model>` tag.

* `<collision>` goes inside of the `<link>` tag. This does not need to provide a name or refer to anything; instead it mirrors the normal `<collision>` element of a normal `<link>` tag.

* `<pose>` can optionally go inside of `<collision>` to give a convex geometry an offset from the link frame.

* `<geometry>` goes inside of `<collision>` and mirrors the normal `<geometry>` tag. It can only accept the following tags:
   * `<box>` which can accept a `<size>` tag that takes three floating point values (for x/y/z). This also accepts a `<dilation>` tag.
   * `<cylinder>` which can accept a `<radius>` and a `<length>` tag. This also accepts a `<dilation>` tag.
   * `<capsule>` which can accept a `<radius>` and a `<length>` tag.
   * `<sphere>` which can accept a `<radius>` tag.

* Optionally, you can specify a `<range>` tag inside of the `<pair>` element. This accepts a single floating-point value which effectively specifies the maximum distance from each object that we'll calculate. If objects are separated by a greater distance than `<range>`, then their distance will not be calculated. The default for this value is 10 meters.
