## Fault-tolerant path planning in Frenet frame

A general-purpose path server in Frenet frame. This work-in-progress powers the autonomous Indycar for @BlackAndGoldAutonomousRacing.

With a frenet frame representation, the search space for trajectory planning is significantly reduced compared to 2-D costmap approach, especially for non-holonomic and high-speed robots moving through elongated and curved spaces.

It adopts a modular, plugin-based composition of functionalities, and supports coherent evaluation of tightly-coupled cost functions.

The third highlight in this work involves the fault tolerance of path references. By correlating the path representation in multiple reference frames, it aims at providing reliable trajectory planning under multiple unstable sensor / state estimation readouts (e.g. scattered GNSS service).

More technical details can be found in the design document.
