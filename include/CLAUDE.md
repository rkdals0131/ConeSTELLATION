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

## Current Status

- Basic structure established
- Core modules implemented (preprocessing, mapping, factors)
- Missing odometry and async modules