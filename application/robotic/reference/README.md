# Reference data

The table here checks simon's compiled models against
[MuJoCo](https://mujoco.org) (Apache-2.0) on the models in `../models`:

| Model | Shows |
|---|---|
| `pendulum.xml` | A capsule and a sphere on a hinge |
| `double_pendulum.xml` | Two capsules on hinges, one by density and one by mass, the upper turned by Euler angles |
| `cartpole.xml` | A box on a slide with limits and damping, a capsule on a hinge with armature, a motor with a gear and a control range |
| `free_body.xml` | A free body of a box, a cylinder and an ellipsoid, each offset and turned, its inertia combined on principal axes |
| `features.xml` | Default classes nested and inherited, childclass, partial vectors, angles in radians, fixed-axes Euler angles, every orientation, explicit diagonal and full inertia, slide, ball and hinge joints, limits off and on, references |
| `boxes.xml` | Five boxes stacked on a plane, each free |

| Script | Writes | Read by |
|---|---|---|
| `mujoco_models.py` | `mujoco_models.csv`: every compiled field of every model | `model_test` |

`pip install mujoco` (3.14.0) runs the script.
