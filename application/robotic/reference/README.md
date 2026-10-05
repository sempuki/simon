# Reference data

The tables here check simon's models, dynamics and control against
[MuJoCo](https://mujoco.org) (Apache-2.0) on the models in `../models`:

| Model | Shows |
|---|---|
| `pendulum.xml` | A capsule and a sphere on a hinge |
| `double_pendulum.xml` | Two capsules on hinges, one by density and one by mass, the upper turned by Euler angles |
| `cartpole.xml` | A box on a slide with limits and damping, a capsule on a hinge with armature, a motor with a gear and a control range |
| `free_body.xml` | A free body of a box, a cylinder and an ellipsoid, each offset and turned, its inertia combined on principal axes |
| `features.xml` | Default classes nested and inherited, childclass, partial vectors, angles in radians, fixed-axes Euler angles, every orientation, explicit diagonal and full inertia, slide, ball and hinge joints, limits off and on, references |
| `boxes.xml` | Five boxes stacked on a plane, each free |
| `arm.xml` | Four joints under position, velocity and general actuators, a default class giving a servo's gain |

| Script | Writes | Read by |
|---|---|---|
| `mujoco_models.py` | `mujoco_models.csv`: every compiled field of every model | `model_test` |
| `mujoco_dynamics.py` | `mujoco_dynamics.csv`: positions and velocities at every step of six cases, constraints off | `dynamics_test` |
| `mujoco_control.py` | `mujoco_control.csv`: the arm and the balanced cart-pole at every step; `mujoco_feedback.csv`: the cart-pole's regulator, from MuJoCo's linearization | `control_test` |

`pip install mujoco numpy` (MuJoCo 3.14.0) runs the scripts.
