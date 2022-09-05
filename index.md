## Welcome to the project of Frenet/Cartesian lifting for obstacle avoidance within nonlinear MPC

This work is related to an ACC 2023 submission by Rudolf Reiter.

In the following we see three different MPC formulations of obstacle avoidance. In the first animation we see a naive formulation of the obstacle constraints in the Frenet frame where obstacle deformations are ignored. Due to the wrong constraints this formulation leads to a crash. In the second animation the obstalce avoidance in the Frenet frame is over-approximated, which leads to collision avoidance but unnecessary defensive behavior. In the third video we show the porposed lifting algorithm, where the obstacle avoidance is formulated in the Cartesian coordinate system, whereas the road constraints and progress are formulated in the Frenet coordinate frame.

## MPC obstacle avoidance without lifting
In the following rendered simulation, the collision avoidance constraints are naively formulated in the Frenet frame. On the left, the Cartesian coordinate frame is shown, on the right the Frenet frame is shown, which is a road aligned coordinate system. The shape of the obstacle is assumed to be the same in the Frenet frame and formulated by simple collision avoidance constraints emerging from the Cartesian frame. The resulting optimization problem does not capture the true shape and consequently, a crash occurs.
<iframe width="1200" height="600" src="https://www.youtube.com/embed/z-pKW0YeRaU" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## MPC obstacle avoidance without lifting and increased safety distances
A simple fix of the above collision due to the wrong captured shape are over-approximations of the non-convex obstacle shape. Obviously, the over-approximation is restrictive and leads to a conservative behavior.
<iframe width="1200" height="600" src="https://www.youtube.com/embed/9h2oaFButNA" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## MPC obstacle avoidance with lifting
The following rendering of a simulation of the proposed Frenet-Cartesian lifting algorithm shows a successfull overtaking maneuver. The obstacle avoidance constraints are formulated in the Cartesian frame, whereas the boundary constraints and reference tracking are formulated in the Frenet frame. It can be seen how a simple Cartesian obstacle shape (rectangle) is transformed to a non-convex shape in the Frenet frame, thus making its constraint formulation within the MPC formulation more challenging.
<iframe width="1200" height="600" src="https://www.youtube.com/embed/I2FuciVrCw0" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>


