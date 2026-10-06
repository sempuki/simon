// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows MuJoCo 3.14.0, Copyright 2021 DeepMind Technologies Limited,
// Apache-2.0; translated to C++ and changed. See NOTICE.md.

#include "model/articulated.hpp"

namespace simon::model {

auto find_trees(const ArticulatedModel& model) -> std::vector<Tree> {
  std::vector<Tree> trees;
  for (std::uint32_t b = 1; b < model.bodies.size(); ++b) {
    const ArticulatedBody& body = model.bodies[b];
    if (body.parent == 0) {
      trees.push_back(Tree{.first_body = b,
                           .first_joint = body.first_joint,
                           .first_dof = body.first_dof,
                           .first_geom = body.first_geom});
    }
    Tree& tree = trees.back();
    ++tree.bodies;
    tree.joints += body.joints;
    tree.dofs += body.dofs;
    tree.geoms += body.geoms;
  }
  for (Tree& tree : trees) {
    tree.first_qpos = tree.joints > 0 ? model.joints[tree.first_joint].qpos : 0;
    for (std::uint32_t j = tree.first_joint; j < tree.first_joint + tree.joints;
         ++j) {
      tree.qpos += model.joints[j].type == JointType::FREE   ? 7
                   : model.joints[j].type == JointType::BALL ? 4
                                                             : 1;
    }
    for (std::uint32_t a = 0; a < model.actuators.size(); ++a) {
      std::uint32_t joint = model.actuators[a].joint;
      if (joint >= tree.first_joint && joint < tree.first_joint + tree.joints) {
        tree.actuators.push_back(a);
      }
    }
  }
  return trees;
}

}  // namespace simon::model
