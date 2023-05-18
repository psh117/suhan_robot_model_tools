/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Zachary Kingston */

#include "constraints/kinematics_constraint_functions.h"

#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/math/constants/constants.hpp>

#include <utility>

namespace ob = ompl::base;
namespace ot = ompl::tools;

class ParallelBase
{
public:
    virtual void getStart(Eigen::Ref<Eigen::VectorXd> x) = 0;
    virtual void getGoal(Eigen::Ref<Eigen::VectorXd> x) = 0;
};

class ParallelChain : public ob::Constraint, public ParallelBase
{
public:
    ParallelChain(const unsigned int n, Eigen::Vector3d offset, unsigned int links, unsigned int chainNum,
                  double length = 1);

    void getStart(Eigen::Ref<Eigen::VectorXd> x) override;
    void getGoal(Eigen::Ref<Eigen::VectorXd> x) override;

    Eigen::Ref<const Eigen::VectorXd> getLink(const Eigen::VectorXd &x, const unsigned int idx) const;

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;
    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override;


private:
    const Eigen::Vector3d offset_;
    const unsigned int links_;
    const unsigned int chainNum_;
    const double length_;
};

class ParallelPlatform : public ob::Constraint, public ParallelBase
{
public:
    ParallelPlatform(unsigned int links, unsigned int chains, double radius = 1);

    Eigen::Ref<const Eigen::VectorXd> getTip(const Eigen::VectorXd &x, unsigned int id) const;

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;

    void getStart(Eigen::Ref<Eigen::VectorXd> ) override;
    void getGoal(Eigen::Ref<Eigen::VectorXd> ) override;

private:
    const unsigned int links_;
    const unsigned int chains_;
    const double radius_;
};

class ParallelConstraint : public ob::ConstraintIntersection, public ParallelBase
{
public:
    ParallelConstraint(unsigned int links, unsigned int chains, double radius = 1, double length = 1,
                       double jointRadius = 0.2);

    void getStart(Eigen::Ref<Eigen::VectorXd> x) override;
    void getGoal(Eigen::Ref<Eigen::VectorXd> x) override;

    bool project(Eigen::Ref<Eigen::VectorXd> x) const override;

    ob::StateSpacePtr createSpace() const;
    bool isValid(const Eigen::Ref<const Eigen::VectorXd> &x) const;
    void setEarlyStopping(bool enable);

    /** Create a projection evaluator for the parallel constraint. Finds the
     * centroid of the platform and project it to a one-dimensional space. */
    ob::ProjectionEvaluatorPtr getProjection(ob::StateSpacePtr space) const;

    void dump(std::ofstream &file) const;
    void addBenchmarkParameters(ot::Benchmark *bench) const;

private:
    const unsigned int links_;
    const unsigned int chains_;
    const double radius_;
    const double length_;
    const double jointRadius_;
    bool early_stopping_ {false};
};
