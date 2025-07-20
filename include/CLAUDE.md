# include/ Directory

This directory contains all public headers for the cone_stellation SLAM system, following a header-only design pattern inspired by GLIM.

## Structure

- **cone_stellation/**: Main namespace directory containing all headers
  - **common/**: Core data structures (Cone, EstimationFrame, etc.)
  - **factors/**: GTSAM custom factors for cone SLAM
  - **mapping/**: Mapping and optimization modules
  - **odometry/**: Odometry estimation modules (TO BE IMPLEMENTED)
  - **preprocessing/**: Cone data preprocessing and filtering
  - **util/**: ROS2 utilities and helpers
  - **viewer/**: Visualization components (TO BE IMPLEMENTED)

## Design Philosophy

Following GLIM's approach, most implementations are header-only for:
- Template flexibility
- Easier integration
- No need for complex linking
- Inline optimization opportunities

## Current Status (2025-07-20)

### Recent Updates
- **mapping/data_association.hpp**: ✅ NEW - Basic data association module
  - Nearest neighbor matching with color constraints
  - Configurable association distance threshold
  - Foundation for preventing landmark duplication
- **simple_cone_mapping.hpp**: Integrated data association
  - No longer creates duplicate landmarks
  - Associates observations to existing landmarks
  - Limits new landmark creation per frame
- **odometry/**: Implemented cone-based odometry modules
  - cone_odometry_base.hpp: Abstract base class for odometry estimation
  - cone_odometry_2d.hpp: 2D implementation using GTSAM factor graph
  - async_cone_odometry.hpp: Asynchronous wrapper for real-time processing
- **cone.hpp**: Fixed color definitions to match actual track layout (YELLOW=right, BLUE=left)
- **viewer/**: Separated visualization modules following GLIM architecture

### Fixed Issues
- ✅ **Critical**: Implemented cone-based odometry - no longer relies on TF
- ✅ Data association preventing landmark duplication
- ✅ Motion estimation from cone observations working

### Next Steps
- Build and test the integrated system with data association
- Debug blue cone intermittent appearance issue
- Fine-tune association parameters
- Enable inter-landmark factors