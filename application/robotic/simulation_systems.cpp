// Copyright 2026 -- CONTRIBUTORS. See LICENSE.

#include "application/robotic/simulation_systems.hpp"

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
  place_geoms<SmallCapacity>(world);
  place_geoms<LargeCapacity>(world);

  // The world's geoms and the planes, with every body that may touch them.
  for (std::uint32_t g : mechanics_->unbounded()) {
    std::uint32_t owner = m.geoms[g].body;
    for (std::uint32_t b = 1; b < m.bodies.size(); ++b) {
      if (b == owner || mechanics_->filter().discards(owner, b)) {
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
  if (mechanics_->filter().discards(first, second)) {
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
