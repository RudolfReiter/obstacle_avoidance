## Welcome to the project of Frenet/Cartesian lifting for obstacle avoidance within nonlinear MPC

This work is related to an ACC 2023 submission by Rudolf Reiter.

In the following we see three different MPC formulations of obstacle avoidance. In the first video we show the porposed lifting algorithm, where the obstacle avoidance is formulated in the Cartesian coordinate system, whereas the road constraints and progress are formulated in the Frenet coordinate frame. In the second video we see a naive formulation of the obstacle constraints in the Frenet frame where obstacle deformations are ignored. Due to the wrong constraints this formulation leads to a crash. In the thrid video the obstalce avoidance in the Frenet frame is over-approximated, which leads to collision avoidance but unnecessary defensive behavior.

## MPC obstacle avoidance with lifting
<iframe width="800" height="400" src="https://www.youtube.com/embed/I2FuciVrCw0" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## MPC obstacle avoidance without lifting
<iframe width="800" height="400" src="https://www.youtube.com/embed/z-pKW0YeRaU" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## MPC obstacle avoidance without lifting and increased safety distances
<iframe width="800" height="400" src="https://www.youtube.com/embed/9h2oaFButNA" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>


