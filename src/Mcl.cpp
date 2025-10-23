// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-FileCopyrightText: 2025 Ryo Funai
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "emcl2/Mcl.h"

namespace emcl2
{

Mcl::Mcl(std::vector<Particle> particles)
: particles_(std::move(particles))
{
}

void Mcl::sensorUpdate(const HashedVoxelMap & map, const PointCloudObservation & observation)
{
  for (auto & particle : particles_) {
    particle.updateWeight(map, observation);
  }
}

void Mcl::normalizeWeights()
{
  double sum = 0.0;
  for (const auto & particle : particles_) {
    sum += particle.weight();
  }
  if (sum <= 0.0) {
    return;
  }
  for (auto & particle : particles_) {
    particle.setWeight(particle.weight() / sum);
  }
}

void Mcl::resample(std::mt19937 & rng)
{
  if (particles_.empty()) {
    return;
  }

  double total_weight = 0.0;
  for (const auto & particle : particles_) {
    total_weight += particle.weight();
  }

  if (total_weight <= 0.0) {
    double uniform = 1.0 / static_cast<double>(particles_.size());
    for (auto & particle : particles_) {
      particle.setWeight(uniform);
    }
    return;
  }

  std::vector<double> cumulative(particles_.size());
  double running = 0.0;
  for (std::size_t i = 0; i < particles_.size(); ++i) {
    running += particles_[i].weight();
    cumulative[i] = running;
  }

  std::uniform_real_distribution<double> dist(0.0,
    total_weight / static_cast<double>(particles_.size()));
  double step = total_weight / static_cast<double>(particles_.size());
  double target = dist(rng);

  std::vector<Particle> resampled;
  resampled.reserve(particles_.size());

  std::size_t index = 0;
  for (std::size_t i = 0; i < particles_.size(); ++i) {
    while (target > cumulative[index] && index + 1 < cumulative.size()) {
      ++index;
    }
    resampled.push_back(particles_[index]);
    target += step;
  }

  double uniform = 1.0 / static_cast<double>(resampled.size());
  for (auto & particle : resampled) {
    particle.setWeight(uniform);
  }

  particles_.swap(resampled);
}

void Mcl::initialize(double x, double y, double yaw)
{
  if (particles_.empty()) {
    return;
  }

  double uniform_weight = 1.0 / static_cast<double>(particles_.size());
  for (auto & particle : particles_) {
    particle.pose().set(x, y, yaw);
    particle.setWeight(uniform_weight);
  }
}

}  // namespace emcl2
