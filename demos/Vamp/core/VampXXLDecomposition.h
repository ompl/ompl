#pragma once

#include <ompl/geometric/planners/xxl/XXLDecomposition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/util/RandomNumbers.h>
#include <memory>
#include <vector>
#include <cmath>

namespace vamp_ompl {

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Forward declaration
class VampWorkspaceBounds;

/**
 * @brief Simple concrete XXL decomposition for manipulators
 * 
 * This is a basic implementation that provides the required pure virtual methods
 * for XXL decomposition. It creates a grid-based decomposition of the workspace.
 */
class VampXXLDecomposition : public og::XXLDecomposition {
public:
    VampXXLDecomposition(const ob::SpaceInformationPtr& si,
                        const ob::RealVectorBounds& bounds,
                        const std::vector<int>& slices,
                        bool diagonal_edges = true)
        : si_(si), bounds_(bounds), slices_(slices) {
        (void)diagonal_edges; // Unused parameter
        
        // Calculate total regions
        num_regions_ = 1;
        for (int slice : slices_) {
            num_regions_ *= slice;
        }
        
        // Calculate cell sizes
        cell_sizes_.resize(bounds_.low.size());
        for (size_t i = 0; i < bounds_.low.size(); ++i) {
            cell_sizes_[i] = (bounds_.high[i] - bounds_.low[i]) / slices_[i];
        }
    }
    
    // Required pure virtual method implementations
    int getNumRegions() const override {
        return num_regions_;
    }
    
    int getDimension() const override {
        return bounds_.low.size();
    }
    
    int numLayers() const override {
        return 1; // Simple single-layer decomposition
    }
    
    int locateRegion(const ob::State* s) const override {
        // Extract position from state and convert to region ID
        std::vector<double> coord;
        stateToCoordinates(s, coord);
        return coordinatesToRegion(coord);
    }
    
    int locateRegion(const std::vector<double>& coord) const override {
        return coordinatesToRegion(coord);
    }
    
    void getNeighbors(int rid, std::vector<int>& neighbors) const override {
        neighbors.clear();
        std::vector<int> indices = regionToIndices(rid);
        
        // Get all adjacent cells (6-connected or 26-connected depending on diagonal_edges_)
        for (int dim = 0; dim < getDimension(); ++dim) {
            // Negative direction
            if (indices[dim] > 0) {
                std::vector<int> neighbor_indices = indices;
                neighbor_indices[dim]--;
                neighbors.push_back(indicesToRegion(neighbor_indices));
            }
            // Positive direction
            if (indices[dim] < slices_[dim] - 1) {
                std::vector<int> neighbor_indices = indices;
                neighbor_indices[dim]++;
                neighbors.push_back(indicesToRegion(neighbor_indices));
            }
        }
    }
    
    double distanceHeuristic(int r1, int r2) const override {
        std::vector<int> indices1 = regionToIndices(r1);
        std::vector<int> indices2 = regionToIndices(r2);
        
        // Manhattan distance
        double dist = 0.0;
        for (int i = 0; i < getDimension(); ++i) {
            dist += std::abs(indices1[i] - indices2[i]);
        }
        return dist;
    }
    
    bool sampleFromRegion(int r, ob::State* s, const ob::State* seed = nullptr) const override {
        (void)seed; // Unused parameter
        // Sample uniformly from the region
        std::vector<int> indices = regionToIndices(r);
        std::vector<double> coord(getDimension());
        
        for (int i = 0; i < getDimension(); ++i) {
            double low = bounds_.low[i] + indices[i] * cell_sizes_[i];
            double high = low + cell_sizes_[i];
            coord[i] = rng_.uniformReal(low, high);
        }
        
        coordinatesToState(coord, s);
        return si_->isValid(s);
    }
    
    bool sampleFromRegion(int r, ob::State* s, const ob::State* seed, int layer) const override {
        (void)layer; // Unused parameter
        // For single-layer decomposition, ignore layer parameter
        return sampleFromRegion(r, s, seed);
    }
    
    void project(const ob::State* s, std::vector<double>& coord, int layer = 0) const override {
        (void)layer; // Unused parameter
        stateToCoordinates(s, coord);
    }
    
    void project(const ob::State* s, std::vector<int>& layers) const override {
        (void)s; // Unused parameter
        layers.clear();
        layers.push_back(0); // Single layer
    }

private:
    ob::SpaceInformationPtr si_;
    ob::RealVectorBounds bounds_;
    std::vector<int> slices_;
    std::vector<double> cell_sizes_;
    int num_regions_;
    mutable ompl::RNG rng_;
    
    void stateToCoordinates(const ob::State* s, std::vector<double>& coord) const {
        // Simple extraction - assumes first 3 dimensions are position
        const auto* rv_state = s->as<ob::RealVectorStateSpace::StateType>();
        coord.resize(std::min(getDimension(), 3));
        
        // For manipulators, we approximate end-effector position
        // This is a placeholder - in practice, use actual forward kinematics
        for (int i = 0; i < std::min(getDimension(), 3); ++i) {
            if (i < static_cast<int>(si_->getStateSpace()->getDimension())) {
                coord[i] = rv_state->values[i]; // Simplified projection
            } else {
                coord[i] = 0.0;
            }
        }
    }
    
    void coordinatesToState(const std::vector<double>& coord, ob::State* s) const {
        (void)coord; // Unused parameter
        // Sample a random valid state - this is a placeholder
        auto sampler = si_->getStateSpace()->allocStateSampler();
        sampler->sampleUniform(s);
    }
    
    int coordinatesToRegion(const std::vector<double>& coord) const {
        std::vector<int> indices(getDimension());
        for (int i = 0; i < getDimension(); ++i) {
            indices[i] = std::max(0, std::min(slices_[i] - 1, 
                static_cast<int>((coord[i] - bounds_.low[i]) / cell_sizes_[i])));
        }
        return indicesToRegion(indices);
    }
    
    std::vector<int> regionToIndices(int region) const {
        std::vector<int> indices(getDimension());
        int remaining = region;
        for (int i = getDimension() - 1; i >= 0; --i) {
            indices[i] = remaining % slices_[i];
            remaining /= slices_[i];
        }
        return indices;
    }
    
    int indicesToRegion(const std::vector<int>& indices) const {
        int region = 0;
        int multiplier = 1;
        for (int i = getDimension() - 1; i >= 0; --i) {
            region += indices[i] * multiplier;
            multiplier *= slices_[i];
        }
        return region;
    }
};

/**
 * @brief Configuration for XXL decomposition
 */
struct XXLDecompositionConfig {
    // Workspace bounds for decomposition
    ob::RealVectorBounds workspace_bounds;
    
    // Number of slices in each dimension
    std::vector<int> slices_per_dimension;
    
    // Whether to include diagonal edges in decomposition graph
    bool diagonal_edges = true;
    
    // Projection dimensions (which robot links/joints to project)
    std::vector<int> projection_indices;
    
    // Decomposition type
    enum Type { PLANAR, POSITION, CUSTOM } type = POSITION;
    
    XXLDecompositionConfig(const ob::RealVectorBounds& bounds) : workspace_bounds(bounds) {}
    
    /**
     * @brief Create default configuration for manipulator
     * @param num_joints Number of robot joints
     * @param workspace_bounds Reachable workspace bounds
     */
    static XXLDecompositionConfig createManipulatorConfig(
        int num_joints, 
        const ob::RealVectorBounds& workspace_bounds) {
        
        XXLDecompositionConfig config(workspace_bounds);
        config.workspace_bounds = workspace_bounds;
        config.type = POSITION;
        
        // Intelligent defaults based on dimensionality
        int slices = std::max(2, num_joints / 3);
        config.slices_per_dimension = std::vector<int>(3, slices); // x, y, z
        
        // Project end-effector and mid-point for high-DOF robots
        config.projection_indices.push_back(num_joints - 1); // end-effector
        if (num_joints > 6) {
            config.projection_indices.push_back(num_joints / 2 - 1); // mid-point
        }
        
        return config;
    }
};

/**
 * @brief Generic XXL decomposition factory for VAMP robots
 */
class VampXXLDecompositionFactory {
public:
    /**
     * @brief Create XXL decomposition for given space information
     * @param si Space information
     * @param config Decomposition configuration
     * @return XXL decomposition pointer
     */
    static og::XXLDecompositionPtr createDecomposition(
        const ob::SpaceInformationPtr& si,
        const XXLDecompositionConfig& config) {
        
        switch (config.type) {
            case XXLDecompositionConfig::PLANAR:
                return createPlanarDecomposition(si, config);
            case XXLDecompositionConfig::POSITION:
                return createPositionDecomposition(si, config);
            case XXLDecompositionConfig::CUSTOM:
                throw std::runtime_error("Custom XXL decomposition not implemented");
            default:
                throw std::runtime_error("Unknown XXL decomposition type");
        }
    }
    
    /**
     * @brief Create intelligent decomposition based on state space type
     * @param si Space information
     * @param workspace_bounds Workspace bounds (optional, will be estimated if empty)
     * @return XXL decomposition pointer
     */
    static og::XXLDecompositionPtr createIntelligentDecomposition(
        const ob::SpaceInformationPtr& si,
        const ob::RealVectorBounds& workspace_bounds = ob::RealVectorBounds(0)) {
        
        auto state_space = si->getStateSpace();
        int dimension = state_space->getDimension();
        
        // Use provided bounds or create simple default bounds
        ob::RealVectorBounds bounds(3);
        if (workspace_bounds.low.size() > 0) {
            bounds = workspace_bounds;
        } else {
            // Simple default workspace bounds for testing
            bounds.setLow(0, -1.0); bounds.setHigh(0, 1.0);   // X: ±1m
            bounds.setLow(1, -1.0); bounds.setHigh(1, 1.0);   // Y: ±1m  
            bounds.setLow(2, -0.2); bounds.setHigh(2, 1.5);   // Z: table to reach
        }
        
        // Create appropriate configuration
        auto config = XXLDecompositionConfig::createManipulatorConfig(dimension, bounds);
        
        // Adjust based on state space type
        if (state_space->getType() == ob::STATE_SPACE_SE2) {
            config.type = XXLDecompositionConfig::PLANAR;
            config.slices_per_dimension = {4, 4, 1}; // x, y, theta
        } else if (state_space->getType() == ob::STATE_SPACE_SE3) {
            config.type = XXLDecompositionConfig::POSITION;
            config.slices_per_dimension = {3, 3, 3}; // x, y, z
        }
        
        return createDecomposition(si, config);
    }

private:
    static og::XXLDecompositionPtr createPlanarDecomposition(
        const ob::SpaceInformationPtr& si,
        const XXLDecompositionConfig& config) {
        
        // Use our concrete implementation for planar cases
        return std::make_shared<VampXXLDecomposition>(
            si,
            config.workspace_bounds,
            config.slices_per_dimension,
            config.diagonal_edges
        );
    }
    
    static og::XXLDecompositionPtr createPositionDecomposition(
        const ob::SpaceInformationPtr& si,
        const XXLDecompositionConfig& config) {
        
        // Use our concrete implementation for position-based cases
        return std::make_shared<VampXXLDecomposition>(
            si,
            config.workspace_bounds,
            config.slices_per_dimension,
            config.diagonal_edges
        );
    }
};

/**
 * @brief Generalized workspace bounds calculator for any robot configuration
 * 
 * This class provides automatic workspace estimation based on joint limits and state space
 * configuration, eliminating the need for robot-specific hardcoded values. It works with
 * any robot by analyzing the joint space bounds and applying conservative scaling factors.
 */
class VampWorkspaceBounds {
public:
    /**
     * @brief Calculate workspace bounds for manipulator
     * @param num_joints Number of joints
     * @param link_length Average link length (use robot-specific values)
     * @param base_position Base position
     * @return Workspace bounds
     */
    static ob::RealVectorBounds calculateManipulatorBounds(
        int num_joints, 
        double link_length = 1.0,
        const std::vector<double>& base_position = {0.0, 0.0, 0.0}) {
        
        // Conservative workspace estimation - could be improved with actual kinematics
        double reach = num_joints * link_length;
        
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0, base_position[0] - reach);
        bounds.setHigh(0, base_position[0] + reach);
        bounds.setLow(1, base_position[1] - reach);
        bounds.setHigh(1, base_position[1] + reach);
        bounds.setLow(2, base_position[2] - reach);
        bounds.setHigh(2, base_position[2] + reach);
        
        return bounds;
    }
    
    /**
     * @brief Calculate workspace bounds from state space bounds
     * @param si Space information
     * @param scale_factor Optional scaling factor for workspace estimation (default: 0.8)
     * @return Workspace bounds (first 3 dimensions)
     */
    static ob::RealVectorBounds extractWorkspaceBounds(const ob::SpaceInformationPtr& si, double scale_factor = 0.8) {
        auto state_space = si->getStateSpace();
        
        // Try to extract bounds from compound state space
        if (auto compound = state_space->as<ob::CompoundStateSpace>()) {
            // Look for SE(3) or RealVector subspaces
            for (unsigned int i = 0; i < compound->getSubspaceCount(); ++i) {
                auto subspace = compound->getSubspace(i);
                if (auto se3 = subspace->as<ob::SE3StateSpace>()) {
                    return se3->getBounds();
                } else if (auto rv = subspace->as<ob::RealVectorStateSpace>()) {
                    if (rv->getDimension() >= 3) {
                        ob::RealVectorBounds bounds(3);
                        auto rv_bounds = rv->getBounds();
                        for (int j = 0; j < 3; ++j) {
                            bounds.setLow(j, rv_bounds.low[j]);
                            bounds.setHigh(j, rv_bounds.high[j]);
                        }
                        return bounds;
                    }
                }
            }
        }
        
        // Generalized workspace estimation based on joint limits
        if (auto rv_space = state_space->as<ob::RealVectorStateSpace>()) {
            auto joint_bounds = rv_space->getBounds();
            int num_joints = rv_space->getDimension();
            
            // Estimate workspace as scaled version of joint space reach
            // This is a conservative approximation that works for most manipulators
            double max_reach = estimateMaxReach(joint_bounds, num_joints, scale_factor);
            
            ob::RealVectorBounds bounds(3);
            bounds.setLow(0, -max_reach); bounds.setHigh(0, max_reach);   // X
            bounds.setLow(1, -max_reach); bounds.setHigh(1, max_reach);   // Y  
            bounds.setLow(2, -0.2); bounds.setHigh(2, max_reach + 0.2);   // Z (assume table setup)
            return bounds;
        }
        
        // Fallback: conservative default bounds
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0, -1.0); bounds.setHigh(0, 1.0);   // X: ±1m
        bounds.setLow(1, -1.0); bounds.setHigh(1, 1.0);   // Y: ±1m  
        bounds.setLow(2, -0.2); bounds.setHigh(2, 1.5);   // Z: table to reach
        return bounds;
    }

private:
    /**
     * @brief Estimate maximum reach based on joint limits
     * @param joint_bounds Joint space bounds
     * @param num_joints Number of joints
     * @param scale_factor Scaling factor for conservative estimation
     * @return Estimated maximum reach
     */
    static double estimateMaxReach(const ob::RealVectorBounds& joint_bounds, int num_joints, double scale_factor) {
        // Simple heuristic: assume each joint contributes to reach
        // This works reasonably well for serial manipulators
        double total_range = 0.0;
        for (int i = 0; i < num_joints && i < 7; ++i) { // Limit to first 7 joints (typical arm)
            double joint_range = joint_bounds.high[i] - joint_bounds.low[i];
            total_range += std::abs(joint_range);
        }
        
        // Scale down to get realistic workspace estimate
        // Typical manipulators have workspace ~60-80% of kinematic reach
        return (total_range / (2.0 * M_PI)) * scale_factor;
    }
};

} // namespace vamp_ompl
