#pragma once

#include <ompl/geometric/planners/xxl/XXLDecomposition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/RandomNumbers.h>
#include <memory>
#include <vector>
#include <cmath>
#include <stdexcept>

namespace vamp_ompl {

namespace ob = ompl::base;
namespace og = ompl::geometric;

/**
 * @brief XXL decomposition for VAMP manipulators
 * 
 */
class VampXXLDecomposition : public og::XXLDecomposition {
public:
    /**
     * @brief Forward kinematics function type: joint_angles -> workspace_position
     */
    using ForwardKinematicsFunction = std::function<void(const double*, std::vector<double>&)>;

    /**
     * @brief Inverse kinematics function type: workspace_position + seed -> joint_configuration
     */
    using InverseKinematicsFunction = std::function<bool(const std::vector<double>&, const double*, double*)>;

    /**
     * @brief Constructor for production use with custom kinematics
     * @param si Space information (must be RealVectorStateSpace)
     * @param workspace_bounds 3D workspace bounds (X, Y, Z)
     * @param slices Number of grid slices per dimension [nx, ny, nz]
     * @param fk_function Forward kinematics function
     * @param ik_function Optional inverse kinematics function
     */
    VampXXLDecomposition(const ob::SpaceInformationPtr& si,
                        const ob::RealVectorBounds& workspace_bounds,
                        const std::vector<int>& slices,
                        ForwardKinematicsFunction fk_function,
                        InverseKinematicsFunction ik_function = nullptr)
        : si_(si)
        , workspace_bounds_(workspace_bounds)
        , slices_(slices)
        , fk_function_(std::move(fk_function))
        , ik_function_(std::move(ik_function)) {
        
        validateInputs();
        initializeDecomposition();
    }

    /**
     * @brief Constructor with default simple kinematic chain for prototyping
     * @param si Space information (must be RealVectorStateSpace)
     * @param workspace_bounds 3D workspace bounds
     * @param slices Number of grid slices per dimension
     * @param link_length Average link length for kinematic chain approximation
     */
    VampXXLDecomposition(const ob::SpaceInformationPtr& si,
                        const ob::RealVectorBounds& workspace_bounds,
                        const std::vector<int>& slices,
                        double link_length = 0.3)
        : VampXXLDecomposition(si, workspace_bounds, slices, 
                              createDefaultForwardKinematics(si, link_length)) {
    }

    // XXLDecomposition interface implementation
    int getNumRegions() const override {
        return num_regions_;
    }
    
    int getDimension() const override {
        return 3; // Always 3D workspace (X, Y, Z)
    }
    
    int numLayers() const override {
        return 1; // Single-layer decomposition
    }
    
    int locateRegion(const ob::State* state) const override {
        std::vector<double> workspace_pos(3);
        projectStateToWorkspace(state, workspace_pos);
        return workspaceToRegion(workspace_pos);
    }
    
    int locateRegion(const std::vector<double>& workspace_coord) const override {
        return workspaceToRegion(workspace_coord);
    }
    
    void getNeighbors(int region_id, std::vector<int>& neighbors) const override {
        neighbors.clear();
        std::vector<int> grid_indices = regionToGridIndices(region_id);
        
        // 6-connected neighbors (face-adjacent only for efficiency)
        for (int dim = 0; dim < 3; ++dim) {
            // Negative direction
            if (grid_indices[dim] > 0) {
                std::vector<int> neighbor_indices = grid_indices;
                neighbor_indices[dim]--;
                neighbors.push_back(gridIndicesToRegion(neighbor_indices));
            }
            // Positive direction
            if (grid_indices[dim] < slices_[dim] - 1) {
                std::vector<int> neighbor_indices = grid_indices;
                neighbor_indices[dim]++;
                neighbors.push_back(gridIndicesToRegion(neighbor_indices));
            }
        }
    }
    
    double distanceHeuristic(int region1, int region2) const override {
        std::vector<int> indices1 = regionToGridIndices(region1);
        std::vector<int> indices2 = regionToGridIndices(region2);
        
        // Euclidean distance in grid space
        double dist_squared = 0.0;
        for (int i = 0; i < 3; ++i) {
            double diff = indices1[i] - indices2[i];
            dist_squared += diff * diff;
        }
        return std::sqrt(dist_squared);
    }
    
    bool sampleFromRegion(int region_id, ob::State* state, const ob::State* seed = nullptr) const override {
        // Sample workspace position from region
        std::vector<double> workspace_pos(3);
        sampleWorkspaceFromRegion(region_id, workspace_pos);
        
        // Convert workspace position to joint configuration
        return workspaceToState(workspace_pos, state, seed);
    }
    
    bool sampleFromRegion(int region_id, ob::State* state, const ob::State* seed, int layer) const override {
        (void)layer; // Single layer decomposition
        return sampleFromRegion(region_id, state, seed);
    }
    
    void project(const ob::State* state, std::vector<double>& workspace_coord, int layer = 0) const override {
        (void)layer; // Single layer decomposition
        projectStateToWorkspace(state, workspace_coord);
    }
    
    void project(const ob::State* state, std::vector<int>& layers) const override {
        (void)state; // Single layer decomposition
        layers.clear();
        layers.push_back(0);
    }

private:
    // Core data members
    ob::SpaceInformationPtr si_;
    ob::RealVectorBounds workspace_bounds_;
    std::vector<int> slices_;
    std::vector<double> cell_sizes_;
    int num_regions_;
    int num_joints_;
    
    // Kinematics functions
    ForwardKinematicsFunction fk_function_;
    InverseKinematicsFunction ik_function_;
    
    // Temporary workspace for computations
    mutable ompl::RNG rng_;
    mutable std::vector<double> temp_workspace_;
    mutable std::vector<double> temp_joints_;

    void validateInputs() {
        if (!si_) {
            throw std::invalid_argument("SpaceInformation cannot be null");
        }
        
        auto rv_space = std::dynamic_pointer_cast<ob::RealVectorStateSpace>(si_->getStateSpace());
        if (!rv_space) {
            throw std::invalid_argument("VampXXLDecomposition requires RealVectorStateSpace");
        }
        
        if (workspace_bounds_.low.size() != 3 || workspace_bounds_.high.size() != 3) {
            throw std::invalid_argument("Workspace bounds must be 3-dimensional");
        }
        
        if (slices_.size() != 3) {
            throw std::invalid_argument("Slices must be 3-dimensional");
        }
        
        for (int slice : slices_) {
            if (slice <= 0) {
                throw std::invalid_argument("All slice counts must be positive");
            }
        }
        
        if (!fk_function_) {
            throw std::invalid_argument("Forward kinematics function cannot be null");
        }
    }
    
    void initializeDecomposition() {
        num_joints_ = si_->getStateSpace()->getDimension();
        
        // Calculate total regions and cell sizes
        num_regions_ = slices_[0] * slices_[1] * slices_[2];
        
        cell_sizes_.resize(3);
        for (int i = 0; i < 3; ++i) {
            cell_sizes_[i] = (workspace_bounds_.high[i] - workspace_bounds_.low[i]) / slices_[i];
        }
        
        // Initialize temporary storage
        temp_workspace_.resize(3);
        temp_joints_.resize(num_joints_);
    }
    
    void projectStateToWorkspace(const ob::State* state, std::vector<double>& workspace_pos) const {
        const auto* rv_state = state->as<ob::RealVectorStateSpace::StateType>();
        fk_function_(rv_state->values, workspace_pos);
    }
    
    int workspaceToRegion(const std::vector<double>& workspace_pos) const {
        std::vector<int> grid_indices(3);
        for (int i = 0; i < 3; ++i) {
            // Clamp to valid range and convert to grid index
            double clamped = std::max(workspace_bounds_.low[i], 
                                    std::min(workspace_bounds_.high[i], workspace_pos[i]));
            int index = static_cast<int>((clamped - workspace_bounds_.low[i]) / cell_sizes_[i]);
            grid_indices[i] = std::max(0, std::min(slices_[i] - 1, index));
        }
        return gridIndicesToRegion(grid_indices);
    }
    
    std::vector<int> regionToGridIndices(int region_id) const {
        std::vector<int> indices(3);
        int remaining = region_id;
        indices[2] = remaining % slices_[2]; remaining /= slices_[2];
        indices[1] = remaining % slices_[1]; remaining /= slices_[1];
        indices[0] = remaining;
        return indices;
    }
    
    int gridIndicesToRegion(const std::vector<int>& indices) const {
        return indices[0] * slices_[1] * slices_[2] + indices[1] * slices_[2] + indices[2];
    }
    
    void sampleWorkspaceFromRegion(int region_id, std::vector<double>& workspace_pos) const {
        std::vector<int> indices = regionToGridIndices(region_id);
        
        for (int i = 0; i < 3; ++i) {
            double low = workspace_bounds_.low[i] + indices[i] * cell_sizes_[i];
            double high = low + cell_sizes_[i];
            workspace_pos[i] = rng_.uniformReal(low, high);
        }
    }
    
    bool workspaceToState(const std::vector<double>& workspace_pos, ob::State* state, const ob::State* seed) const {
        // Use inverse kinematics if available
        if (ik_function_ && seed) {
            const auto* seed_rv = seed->as<ob::RealVectorStateSpace::StateType>();
            auto* state_rv = state->as<ob::RealVectorStateSpace::StateType>();
            
            if (ik_function_(workspace_pos, seed_rv->values, state_rv->values)) {
                si_->getStateSpace()->enforceBounds(state);
                return si_->isValid(state);
            }
        }
        
        // Fallback: sample random valid configuration
        // This is not ideal but allows XXL to function without proper IK
        auto sampler = si_->getStateSpace()->allocStateSampler();
        for (int attempt = 0; attempt < 50; ++attempt) {
            sampler->sampleUniform(state);
            if (si_->isValid(state)) {
                return true;
            }
        }
        
        return false;
    }
    
    static ForwardKinematicsFunction createDefaultForwardKinematics(const ob::SpaceInformationPtr& si, double link_length) {
        int num_joints = si->getStateSpace()->getDimension();
        
        return [num_joints, link_length](const double* joint_values, std::vector<double>& workspace_pos) {
            workspace_pos.resize(3);
            
            // Simple planar kinematic chain approximation
            double x = 0.0, y = 0.0, z = 0.4; // Start at reasonable height
            double angle_sum = 0.0;
            
            for (int i = 0; i < num_joints; ++i) {
                angle_sum += joint_values[i];
                x += link_length * cos(angle_sum);
                y += link_length * sin(angle_sum);
            }
            
            workspace_pos[0] = x;
            workspace_pos[1] = y;
            workspace_pos[2] = z;
        };
    }
};

/**
 * @brief factory for XXL decomposition
 * 
 * This factory provides sensible defaults
 * while allowing full customization.
 */
class VampXXLDecompositionFactory {
public:
    /**
     * @brief Create XXL decomposition with default settings
     * @param si Space information (must be RealVectorStateSpace)
     * @param workspace_bounds 3D workspace bounds (optional, will be estimated)
     * @return XXL decomposition ready
     */
    static og::XXLDecompositionPtr createDefault(
        const ob::SpaceInformationPtr& si,
        const ob::RealVectorBounds& workspace_bounds = getDefaultWorkspaceBounds()) {
        
        // Verify RealVectorStateSpace requirement
        if (!std::dynamic_pointer_cast<ob::RealVectorStateSpace>(si->getStateSpace())) {
            throw std::invalid_argument("VampXXLDecomposition requires RealVectorStateSpace");
        }
        
        int num_joints = si->getStateSpace()->getDimension();
        
        // Intelligent grid sizing based on dimensionality
        int slices_per_dim = std::max(3, std::min(8, num_joints / 2));
        std::vector<int> slices = {slices_per_dim, slices_per_dim, slices_per_dim};
        
        return std::make_shared<VampXXLDecomposition>(si, workspace_bounds, slices);
    }
    
    /**
     * @brief Create XXL decomposition with custom kinematics
     * @param si Space information
     * @param workspace_bounds 3D workspace bounds
     * @param slices Grid resolution per dimension
     * @param fk_function Forward kinematics function
     * @param ik_function Optional inverse kinematics function
     * @return Custom XXL decomposition
     */
    static og::XXLDecompositionPtr createCustom(
        const ob::SpaceInformationPtr& si,
        const ob::RealVectorBounds& workspace_bounds,
        const std::vector<int>& slices,
        VampXXLDecomposition::ForwardKinematicsFunction fk_function,
        VampXXLDecomposition::InverseKinematicsFunction ik_function = nullptr) {
        
        return std::make_shared<VampXXLDecomposition>(
            si, workspace_bounds, slices, fk_function, ik_function);
    }

private:
    /**
     * @brief Get reasonable default workspace bounds for most manipulators
     */
    static ob::RealVectorBounds getDefaultWorkspaceBounds() {
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0, -1.0);  bounds.setHigh(0, 1.0);   // X: ±1m
        bounds.setLow(1, -1.0);  bounds.setHigh(1, 1.0);   // Y: ±1m  
        bounds.setLow(2, 0.0);   bounds.setHigh(2, 1.5);   // Z: 0 to 1.5m
        return bounds;
    }
};

} // namespace vamp_ompl
