// Copyright 2026 -- CONTRIBUTORS. See LICENSE.
// Follows MuJoCo 3.14.0, Copyright 2021 DeepMind Technologies Limited,
// Apache-2.0; translated to C++ and changed. See NOTICE.md.

#include "application/robotic/simulation_systems.hpp"

#include <cmath>
#include <numbers>

namespace simon::robotic {

auto Collide::prepare(SystemWorld& world) -> bool {
  const model::ArticulatedModel& m = mechanics_->model();
  const std::vector<model::Tree>& trees = mechanics_->trees();
  std::vector<model::Contact>& contacts = contacts_->contacts;
  if (frames_.size() != m.geoms.size()) {
    frames_.assign(m.geoms.size(), {});
    unbounded_.assign(m.geoms.size(), 0);
    for (std::uint32_t g : mechanics_->unbounded()) {
      unbounded_[g] = 1;
    }
    const model::ArticulatedBody& ground = m.bodies[0];
    for (std::uint32_t g = ground.first_geom;
         g < ground.first_geom + ground.geoms; ++g) {
      frames_[g] = model::compute_geom_frame(m.geoms[g], model::BodyFrame{});
    }
    tree_of_.assign(m.bodies.size(), NO_TREE);
    for (std::uint32_t t = 0; t < trees.size(); ++t) {
      for (std::uint32_t b = 0; b < trees[t].bodies; ++b) {
        tree_of_[trees[t].first_body + b] = t;
      }
    }
  }
  touching_.assign(trees.size(), 0);
  contacts.clear();
  place_geoms<SingleCapacity>(world);
  place_geoms<SmallCapacity>(world);
  place_geoms<LargeCapacity>(world);
  place_geoms<HugeCapacity>(world);

  // The world's geoms and the planes, with every body that may touch them.
  for (std::uint32_t g : mechanics_->unbounded()) {
    std::uint32_t owner = m.geoms[g].body;
    for (std::uint32_t b = 1; b < m.bodies.size(); ++b) {
      if (b == owner || mechanics_->filter().discards(owner, b) ||
          mechanics_->excludes(owner, b)) {
        continue;
      }
      const model::ArticulatedBody& body = m.bodies[b];
      for (std::uint32_t other = body.first_geom;
           other < body.first_geom + body.geoms; ++other) {
        if (unbounded_[other] == 0) {
          bool first = owner < b;
          model::append_contacts(m, first ? g : other, first ? other : g,
                                 frames_, mechanics_->radii(), InOut(contacts));
        }
      }
    }
  }

  // A tree's bodies with each other.
  for (const model::Tree& tree : trees) {
    for (std::uint32_t b1 = tree.first_body; b1 < tree.first_body + tree.bodies;
         ++b1) {
      for (std::uint32_t b2 = b1 + 1; b2 < tree.first_body + tree.bodies;
           ++b2) {
        collide_bodies(b1, b2);
      }
    }
  }

  // Trees whose spheres overlap, each pair once.
  const auto& mechanisms = world.template store_of<Mechanism>();
  const auto& bounds = world.template store_of<TreeBound>();
  double largest = 0.0;
  bounds.for_each([&](Entity, const TreeBound& bound) {
    largest = std::max(largest, bound.radius);
  });
  bounds.for_each([&](Entity owner, const TreeBound& bound) {
    std::uint32_t mine = mechanisms.component_of(owner).tree;
    world.within(bound, bound.radius + largest,
                 [&](Entity other, const TreeBound& near) {
                   std::uint32_t theirs = mechanisms.component_of(other).tree;
                   if (theirs > mine &&
                       distance(bound, near) <= bound.radius + near.radius) {
                     collide_trees(mine, theirs);
                   }
                 });
  });

  // In order of their bodies, as MuJoCo orders its body pairs.
  auto bodies_of = [&](const model::Contact& contact) {
    std::uint32_t a = m.geoms[contact.geom[0]].body;
    std::uint32_t b = m.geoms[contact.geom[1]].body;
    return std::pair{std::min(a, b), std::max(a, b)};
  };
  std::ranges::stable_sort(contacts, {}, bodies_of);
  for (const model::Contact& contact : contacts) {
    std::uint32_t a = tree_of_[m.geoms[contact.geom[0]].body];
    std::uint32_t b = tree_of_[m.geoms[contact.geom[1]].body];
    if (a != NO_TREE) {
      ++touching_[a];
    }
    if (b != NO_TREE && b != a) {
      ++touching_[b];
    }
  }
  return true;
}

auto Collide::collide_bodies(std::uint32_t first, std::uint32_t second)
    -> void {
  const model::ArticulatedModel& m = mechanics_->model();
  if (mechanics_->filter().discards(first, second) ||
      mechanics_->excludes(first, second)) {
    return;
  }
  const model::ArticulatedBody& a = m.bodies[first];
  const model::ArticulatedBody& b = m.bodies[second];
  for (std::uint32_t g1 = a.first_geom; g1 < a.first_geom + a.geoms; ++g1) {
    if (unbounded_[g1] != 0) {
      continue;
    }
    for (std::uint32_t g2 = b.first_geom; g2 < b.first_geom + b.geoms; ++g2) {
      if (unbounded_[g2] == 0) {
        model::append_contacts(m, g1, g2, frames_, mechanics_->radii(),
                               InOut(contacts_->contacts));
      }
    }
  }
}

auto Collide::collide_trees(std::uint32_t first, std::uint32_t second) -> void {
  const model::Tree& a = mechanics_->trees()[first];
  const model::Tree& b = mechanics_->trees()[second];
  for (std::uint32_t b1 = a.first_body; b1 < a.first_body + a.bodies; ++b1) {
    for (std::uint32_t b2 = b.first_body; b2 < b.first_body + b.bodies; ++b2) {
      collide_bodies(b1, b2);
    }
  }
}

}  // namespace simon::robotic

namespace simon::robotic {

namespace {

constexpr std::array<double, 5> NO_FRICTION{};

// MuJoCo's quaternion as a rotation vector (mju_quat2Vel, dt 1).
auto convert_to_rotation(model::Quaternion4 quat) -> model::Array3 {
  model::articulated::normalize4(InOut(quat));
  model::Array3 axis{quat[1], quat[2], quat[3]};
  double sin_half = model::articulated::normalize3(InOut(axis));
  double angle = 2 * std::atan2(sin_half, quat[0]);
  if (angle > std::numbers::pi) {
    angle -= 2 * std::numbers::pi;
  }
  return {axis[0] * angle, axis[1] * angle, axis[2] * angle};
}

}  // namespace

auto Solve::find_root(std::uint32_t tree) -> std::uint32_t {
  while (parent_[tree] != tree) {
    parent_[tree] = parent_[parent_[tree]];
    tree = parent_[tree];
  }
  return tree;
}

auto Solve::tree_of_geom(std::uint32_t geom) const -> std::uint32_t {
  const model::ArticulatedModel& m = mechanics_->model();
  std::uint32_t body = m.geoms[geom].body;
  if (body == 0) {
    return NO_TREE;
  }
  const std::vector<model::Tree>& trees = mechanics_->trees();
  auto it = std::ranges::upper_bound(trees, body, {}, &model::Tree::first_body);
  std::uint32_t t = static_cast<std::uint32_t>(it - trees.begin()) - 1;
  return trees[t].dofs > 0 ? t : NO_TREE;
}

auto Solve::tree_of_joint(std::uint32_t joint) const -> std::uint32_t {
  return joint_tree_[joint];
}

auto Solve::tendon_length(std::uint32_t k) const -> double {
  const model::ArticulatedModel& m = mechanics_->model();
  const model::Tendon& tendon = m.tendons[k];
  double length = 0.0;
  for (std::size_t i = 0; i < tendon.joints.size(); ++i) {
    const model::Joint& joint = m.joints[tendon.joints[i]];
    std::uint32_t t = tree_of_joint(tendon.joints[i]);
    const model::Tree& tree = mechanics_->trees()[t];
    length +=
        tendon.coefficients[i] * data_[t].qpos[joint.qpos - tree.first_qpos];
  }
  return length;
}

auto Solve::tendon_reached(std::uint32_t k) const -> bool {
  const model::Tendon& tendon = mechanics_->model().tendons[k];
  if (!tendon.limited) {
    return false;
  }
  double length = tendon_length(k);
  return length - tendon.range[0] < tendon.margin ||
         tendon.range[1] - length < tendon.margin;
}

auto Solve::prepare(SystemWorld& world) -> bool {
  const model::ArticulatedModel& m = mechanics_->model();
  const std::vector<model::Tree>& trees = mechanics_->trees();
  std::size_t count = trees.size();
  if (data_.size() != count) {
    data_.resize(count);
    offset_.resize(count);
    rubs_.assign(count, 0);
    limited_.assign(count, 0);
    joint_tree_.assign(m.joints.size(), NO_TREE);
    tree_tendons_.assign(count, {});
    for (std::uint32_t t = 0; t < count; ++t) {
      for (std::uint32_t j = trees[t].first_joint;
           j < trees[t].first_joint + trees[t].joints; ++j) {
        joint_tree_[j] = t;
      }
    }
    for (std::uint32_t k = 0; k < m.tendons.size(); ++k) {
      tree_tendons_[joint_tree_[m.tendons[k].joints.front()]].push_back(k);
    }
    for (std::uint32_t t = 0; t < count; ++t) {
      const model::Tree& tree = trees[t];
      for (std::uint32_t d = tree.first_dof; d < tree.first_dof + tree.dofs;
           ++d) {
        rubs_[t] |= m.dofs[d].friction_loss != 0 ? 1 : 0;
      }
      for (std::uint32_t j = tree.first_joint;
           j < tree.first_joint + tree.joints; ++j) {
        limited_[t] |=
            m.joints[j].limited && m.joints[j].type != model::JointType::FREE
                ? 1
                : 0;
      }
    }
  }
  islands_.assign(count, Island{});
  gather<SingleCapacity>(world);
  gather<SmallCapacity>(world);
  gather<LargeCapacity>(world);
  gather<HugeCapacity>(world);
  ConstraintSolution& out = *solution_;
  out.qacc.assign(m.dofs.size(), 0.0);
  out.qfrc_constraint.assign(m.dofs.size(), 0.0);
  out.constrained.assign(count, 0);
  out.islands = 0;
  out.rows = 0;
  out.iterations = 0;
  for (std::uint32_t t = 0; t < count; ++t) {
    std::copy_n(data_[t].qacc_smooth, trees[t].dofs,
                out.qacc.begin() + trees[t].first_dof);
  }
  if (!enabled_) {
    return true;
  }

  // Trees with rows of their own: dry friction or a limit reached.
  std::vector<std::uint8_t> marked(rubs_);
  for (std::uint32_t t = 0; t < count; ++t) {
    if (limited_[t] == 0) {
      continue;
    }
    const model::Tree& tree = trees[t];
    for (std::uint32_t j = tree.first_joint;
         j < tree.first_joint + tree.joints && marked[t] == 0; ++j) {
      const model::Joint& joint = m.joints[j];
      if (!joint.limited || joint.type == model::JointType::FREE) {
        continue;
      }
      const double* qpos = data_[t].qpos;
      std::uint32_t q = joint.qpos - tree.first_qpos;
      if (joint.type == model::JointType::BALL) {
        model::Array3 r = convert_to_rotation(
            {qpos[q], qpos[q + 1], qpos[q + 2], qpos[q + 3]});
        double angle = model::articulated::normalize3(InOut(r));
        marked[t] =
            std::max(joint.range[0], joint.range[1]) - angle < joint.margin;
      } else {
        marked[t] = qpos[q] - joint.range[0] < joint.margin ||
                    joint.range[1] - qpos[q] < joint.margin;
      }
    }
  }

  // Islands: trees joined by contacts and tendons, by union and find.
  parent_.resize(count);
  for (std::uint32_t t = 0; t < count; ++t) {
    parent_[t] = t;
  }
  for (std::uint32_t k = 0; k < m.tendons.size(); ++k) {
    const model::Tendon& tendon = m.tendons[k];
    if (tendon.friction_loss == 0 && !tendon_reached(k)) {
      continue;
    }
    std::uint32_t first = NO_TREE;
    for (std::uint32_t j : tendon.joints) {
      std::uint32_t t = tree_of_joint(j);
      marked[t] = 1;
      if (first == NO_TREE) {
        first = t;
      } else {
        std::uint32_t ra = find_root(first);
        std::uint32_t rb = find_root(t);
        parent_[std::max(ra, rb)] = std::min(ra, rb);
      }
    }
  }
  const std::vector<model::Contact>& contacts = contacts_->contacts;
  for (const model::Contact& contact : contacts) {
    if (contact.exclude) {
      continue;
    }
    std::uint32_t a = tree_of_geom(contact.geom[0]);
    std::uint32_t b = tree_of_geom(contact.geom[1]);
    if (a != NO_TREE) {
      marked[a] = 1;
    }
    if (b != NO_TREE) {
      marked[b] = 1;
    }
    if (a != NO_TREE && b != NO_TREE) {
      std::uint32_t ra = find_root(a);
      std::uint32_t rb = find_root(b);
      parent_[std::max(ra, rb)] = std::min(ra, rb);
    }
  }
  std::vector<std::uint32_t> index_of_root(count, Island::NONE);
  std::vector<std::vector<std::uint32_t>> members;
  for (std::uint32_t t = 0; t < count; ++t) {
    if (marked[t] == 0) {
      continue;
    }
    std::uint32_t root = find_root(t);
    if (index_of_root[root] == Island::NONE) {
      index_of_root[root] = static_cast<std::uint32_t>(members.size());
      members.emplace_back();
    }
    members[index_of_root[root]].push_back(t);
    islands_[t].index = index_of_root[root];
  }
  std::vector<std::vector<std::uint32_t>> joined(members.size());
  for (std::uint32_t c = 0; c < contacts.size(); ++c) {
    if (contacts[c].exclude) {
      continue;
    }
    std::uint32_t a = tree_of_geom(contacts[c].geom[0]);
    std::uint32_t t = a != NO_TREE ? a : tree_of_geom(contacts[c].geom[1]);
    if (t != NO_TREE) {
      joined[islands_[t].index].push_back(c);
    }
  }
  for (std::uint32_t i = 0; i < members.size(); ++i) {
    std::uint32_t rows = solve_island(members[i], joined[i]);
    for (std::uint32_t t : members[i]) {
      islands_[t].rows = rows;
    }
  }
  out.islands = static_cast<std::uint32_t>(members.size());
  return true;
}

auto Solve::solve_island(std::span<const std::uint32_t> members,
                         std::span<const std::uint32_t> joined)
    -> std::uint32_t {
  const model::ArticulatedModel& m = mechanics_->model();
  const std::vector<model::Tree>& trees = mechanics_->trees();
  const std::vector<double>& body_weight = mechanics_->body_weight();
  const std::vector<double>& dof_weight = mechanics_->dof_weight();
  const std::vector<double>& tendon_weight = mechanics_->tendon_weight();
  model::ConstraintProblem& p = problem_;

  // The island's dofs, its trees' in order.
  std::vector<std::uint32_t>& offset = offset_;
  std::uint32_t n = 0;
  for (std::uint32_t t : members) {
    offset[t] = n;
    n += trees[t].dofs;
  }
  p.dofs = n;
  p.mass.assign(std::size_t{n} * n, 0.0);
  p.qacc_smooth.clear();
  p.qfrc_smooth.clear();
  p.warmstart.clear();
  std::vector<double>& qvel = scratch_.qvel;
  qvel.clear();
  for (std::uint32_t t : members) {
    const TreeData& data = data_[t];
    std::uint32_t k = trees[t].dofs;
    // M(i, j) for j an ancestor of i or i; the rest are zero.
    for (std::uint32_t i = 0; i < k; ++i) {
      for (std::uint32_t j = 0; j <= i; ++j) {
        double value = data.mass[i * data.stride + j];
        p.mass[(offset[t] + i) * n + offset[t] + j] = value;
        p.mass[(offset[t] + j) * n + offset[t] + i] = value;
      }
    }
    p.qacc_smooth.insert(p.qacc_smooth.end(), data.qacc_smooth,
                         data.qacc_smooth + k);
    p.qfrc_smooth.insert(p.qfrc_smooth.end(), data.qfrc_smooth,
                         data.qfrc_smooth + k);
    p.warmstart.insert(p.warmstart.end(), data.warmstart, data.warmstart + k);
    qvel.insert(qvel.end(), data.qvel, data.qvel + k);
  }
  p.jacobian.clear();
  p.kind.clear();
  p.group.clear();
  p.pos.clear();
  p.margin.clear();
  p.friction_loss.clear();
  p.diagonal.clear();
  p.velocity.clear();
  p.friction.clear();
  p.soft.clear();

  std::vector<double>& row = scratch_.row;
  row.assign(n, 0.0);
  auto append = [&](model::ConstraintKind kind, std::uint32_t group, double pos,
                    double margin, double loss, double diagonal,
                    const std::array<double, 5>& friction,
                    const model::SoftConstraint& soft) {
    p.jacobian.insert(p.jacobian.end(), row.begin(), row.end());
    p.kind.push_back(kind);
    p.group.push_back(group);
    p.pos.push_back(pos);
    p.margin.push_back(margin);
    p.friction_loss.push_back(loss);
    p.diagonal.push_back(diagonal);
    double v = 0.0;
    for (std::uint32_t i = 0; i < n; ++i) {
      v += row[i] * qvel[i];
    }
    p.velocity.push_back(v);
    p.friction.push_back(friction);
    p.soft.push_back(soft);
  };

  // Dry friction, by dof.
  for (std::uint32_t t : members) {
    const model::Tree& tree = trees[t];
    for (std::uint32_t d = tree.first_dof; d < tree.first_dof + tree.dofs;
         ++d) {
      const model::Dof& dof = m.dofs[d];
      if (dof.friction_loss == 0) {
        continue;
      }
      std::ranges::fill(row, 0.0);
      row[offset[t] + d - tree.first_dof] = 1;
      append(model::ConstraintKind::FRICTION, p.rows(), 0.0, 0.0,
             dof.friction_loss, dof_weight[d], NO_FRICTION,
             m.joints[dof.joint].friction);
    }
  }

  // A tendon's row: its coefficients on its joints' dofs.
  auto tendon_row = [&](const model::Tendon& tendon, double scale) {
    std::ranges::fill(row, 0.0);
    for (std::size_t i = 0; i < tendon.joints.size(); ++i) {
      const model::Joint& joint = m.joints[tendon.joints[i]];
      std::uint32_t t = tree_of_joint(tendon.joints[i]);
      row[offset[t] + joint.dof - trees[t].first_dof] +=
          scale * tendon.coefficients[i];
    }
  };
  // The island's tendons, in the model's order.
  std::vector<std::uint32_t>& tendons = scratch_.tendons;
  tendons.clear();
  for (std::uint32_t t : members) {
    tendons.insert(tendons.end(), tree_tendons_[t].begin(),
                   tree_tendons_[t].end());
  }
  std::ranges::sort(tendons);
  for (std::uint32_t k : tendons) {
    const model::Tendon& tendon = m.tendons[k];
    if (tendon.friction_loss == 0) {
      continue;
    }
    tendon_row(tendon, 1.0);
    append(model::ConstraintKind::FRICTION, p.rows(), 0.0, 0.0,
           tendon.friction_loss, tendon_weight[k], NO_FRICTION,
           tendon.friction);
  }

  // Limits, by joint: each side a hinge or slide is within its margin of,
  // and a ball's angle past its largest (mj_instantiateLimit).
  for (std::uint32_t t : members) {
    const model::Tree& tree = trees[t];
    const double* qpos = data_[t].qpos;
    for (std::uint32_t j = tree.first_joint; j < tree.first_joint + tree.joints;
         ++j) {
      const model::Joint& joint = m.joints[j];
      if (!joint.limited || joint.type == model::JointType::FREE) {
        continue;
      }
      std::uint32_t q = joint.qpos - tree.first_qpos;
      std::uint32_t c = offset[t] + joint.dof - tree.first_dof;
      if (joint.type == model::JointType::BALL) {
        model::Array3 r = convert_to_rotation(
            {qpos[q], qpos[q + 1], qpos[q + 2], qpos[q + 3]});
        double angle = model::articulated::normalize3(InOut(r));
        double dist = std::max(joint.range[0], joint.range[1]) - angle;
        if (dist < joint.margin) {
          std::ranges::fill(row, 0.0);
          for (std::uint32_t k = 0; k < 3; ++k) {
            row[c + k] = -r[k];
          }
          append(model::ConstraintKind::LIMIT, p.rows(), dist, joint.margin,
                 0.0, dof_weight[joint.dof], NO_FRICTION, joint.limit);
        }
        continue;
      }
      for (int side = -1; side <= 1; side += 2) {
        double dist = side * (joint.range[(side + 1) / 2] - qpos[q]);
        if (dist < joint.margin) {
          std::ranges::fill(row, 0.0);
          row[c] = -static_cast<double>(side);
          append(model::ConstraintKind::LIMIT, p.rows(), dist, joint.margin,
                 0.0, dof_weight[joint.dof], NO_FRICTION, joint.limit);
        }
      }
    }
  }

  // Then by tendon, each side its length is within its margin of.
  for (std::uint32_t k : tendons) {
    const model::Tendon& tendon = m.tendons[k];
    if (!tendon.limited) {
      continue;
    }
    double length = tendon_length(k);
    for (int side = -1; side <= 1; side += 2) {
      double dist = side * (tendon.range[(side + 1) / 2] - length);
      if (dist < tendon.margin) {
        tendon_row(tendon, -static_cast<double>(side));
        append(model::ConstraintKind::LIMIT, p.rows(), dist, tendon.margin, 0.0,
               tendon_weight[k], NO_FRICTION, tendon.limit);
      }
    }
  }

  // Contacts: the difference of the two bodies' point Jacobians, turned to
  // the contact's frame, a frictionless row or a pyramid's edges in pairs
  // (mj_instantiateContact, mj_diagApprox).
  std::vector<double>& translation = scratch_.translation;
  std::vector<double>& rotation = scratch_.rotation;
  translation.resize(3 * std::size_t{n});
  rotation.resize(3 * std::size_t{n});
  for (std::uint32_t c : joined) {
    const model::Contact& contact = contacts_->contacts[c];
    std::ranges::fill(translation, 0.0);
    std::ranges::fill(rotation, 0.0);
    double moved = 0.0;
    double turned = 0.0;
    for (int side = 0; side < 2; ++side) {
      std::uint32_t body = m.geoms[contact.geom[side]].body;
      moved += body_weight[2 * body];
      turned += body_weight[2 * body + 1];
      std::uint32_t t = tree_of_geom(contact.geom[side]);
      if (t == NO_TREE) {
        continue;
      }
      const model::Tree& tree = trees[t];
      std::uint32_t k = tree.dofs;
      std::vector<double>& jp = scratch_.jp;
      std::vector<double>& jr = scratch_.jr;
      jp.resize(3 * std::size_t{k});
      jr.resize(3 * std::size_t{k});
      model::compute_point_jacobian(m, tree, {data_[t].cdof, k}, *data_[t].com,
                                    body, contact.pos, jp, jr);
      double sign = side == 0 ? -1.0 : 1.0;
      for (std::uint32_t r = 0; r < 3; ++r) {
        for (std::uint32_t i = 0; i < k; ++i) {
          translation[r * n + offset[t] + i] += sign * jp[r * k + i];
          rotation[r * n + offset[t] + i] += sign * jr[r * k + i];
        }
      }
    }
    std::uint32_t dim = contact.dim;
    // Rows in the contact frame: normal, tangents, then torsion and rolling.
    std::vector<double>& framed = scratch_.framed;
    framed.assign(std::size_t{dim} * n, 0.0);
    for (std::uint32_t r = 0; r < dim; ++r) {
      bool turning = r >= 3;
      const std::vector<double>& source = turning ? rotation : translation;
      std::uint32_t axis = turning ? r - 3 : r;
      for (std::uint32_t i = 0; i < n; ++i) {
        framed[r * n + i] = contact.frame[3 * axis] * source[i] +
                            contact.frame[3 * axis + 1] * source[n + i] +
                            contact.frame[3 * axis + 2] * source[2 * n + i];
      }
    }
    std::uint32_t group = p.rows();
    if (dim == 1) {
      std::copy_n(framed.begin(), n, row.begin());
      append(model::ConstraintKind::FRICTIONLESS, group, contact.dist,
             contact.include_margin, 0.0, moved, contact.friction,
             contact.soft);
      continue;
    }
    if (m.physics.cone == model::Physics::Cone::ELLIPTIC) {
      // The normal, then the tangents, torsion and rolling, each its row.
      for (std::uint32_t k = 0; k < dim; ++k) {
        std::copy_n(framed.begin() + std::size_t{k} * n, n, row.begin());
        append(model::ConstraintKind::ELLIPTIC, group,
               k == 0 ? contact.dist : 0.0,
               k == 0 ? contact.include_margin : 0.0, 0.0,
               k < 3 ? moved : turned, contact.friction, contact.soft);
      }
      continue;
    }
    for (std::uint32_t k = 1; k < dim; ++k) {
      double mu = contact.friction[k - 1];
      double diagonal = moved + mu * mu * (k - 1 < 2 ? moved : turned);
      for (double sign : {1.0, -1.0}) {
        for (std::uint32_t i = 0; i < n; ++i) {
          row[i] = framed[i] + sign * mu * framed[k * n + i];
        }
        append(model::ConstraintKind::PYRAMIDAL, group, contact.dist,
               contact.include_margin, 0.0, diagonal, contact.friction,
               contact.soft);
      }
    }
  }

  const model::Physics& physics = m.physics;
  model::ConstraintSettings settings{
      .timestep = physics.timestep,
      .solver = physics.solver,
      .iterations = physics.iterations,
      .tolerance = physics.tolerance,
      .mean_inertia = mechanics_->mean_inertia(),
      .model_dofs = static_cast<std::uint32_t>(m.dofs.size()),
      .impratio = physics.impratio};
  model::solve_constraints(p, settings, Out(answer_));
  ConstraintSolution& out = *solution_;
  for (std::uint32_t t : members) {
    const model::Tree& tree = trees[t];
    for (std::uint32_t i = 0; i < tree.dofs; ++i) {
      out.qacc[tree.first_dof + i] = answer_.qacc[offset[t] + i];
      out.qfrc_constraint[tree.first_dof + i] =
          answer_.qfrc_constraint[offset[t] + i];
    }
    out.constrained[t] = 1;
  }
  out.rows += p.rows();
  out.iterations += answer_.iterations;
  return p.rows();
}

}  // namespace simon::robotic
