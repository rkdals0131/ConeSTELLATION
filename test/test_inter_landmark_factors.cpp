#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/inference/Symbol.h>
#include <iostream>

#include "cone_stellation/factors/inter_landmark_factors.hpp"

using namespace gtsam;
using namespace cone_stellation;

class InterLandmarkFactorsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create noise models
        distance_noise_ = noiseModel::Diagonal::Sigmas(Vector1(0.1));
        line_noise_ = noiseModel::Diagonal::Sigmas(Vector1(0.01));
        angle_noise_ = noiseModel::Diagonal::Sigmas(Vector1(0.05));
    }

    SharedNoiseModel distance_noise_;
    SharedNoiseModel line_noise_;
    SharedNoiseModel angle_noise_;
};

TEST_F(InterLandmarkFactorsTest, ConeDistanceFactor) {
    // Test distance factor between two cones
    NonlinearFactorGraph graph;
    Values initial;
    
    // Create two cones 2 meters apart
    Symbol cone1('L', 1);
    Symbol cone2('L', 2);
    
    Point2 p1(0.0, 0.0);
    Point2 p2(2.0, 0.0);
    
    initial.insert(cone1, p1);
    initial.insert(cone2, p2);
    
    // Add distance factor
    double expected_distance = 2.0;
    graph.add(ConeDistanceFactor(cone1, cone2, expected_distance, distance_noise_));
    
    // Optimize
    LevenbergMarquardtParams params;
    LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    Values result = optimizer.optimize();
    
    // Check result
    Point2 optimized_p1 = result.at<Point2>(cone1);
    Point2 optimized_p2 = result.at<Point2>(cone2);
    double actual_distance = (optimized_p2 - optimized_p1).norm();
    
    EXPECT_NEAR(actual_distance, expected_distance, 0.01);
}

TEST_F(InterLandmarkFactorsTest, ConeLineFactor) {
    // Test collinearity factor for three cones
    // For now, we'll just verify the factor evaluates correctly
    
    // Create three cones
    Symbol cone1('L', 1);
    Symbol cone2('L', 2);
    Symbol cone3('L', 3);
    
    // Create the line factor
    ConeLineFactor factor(cone1, cone2, cone3, line_noise_);
    
    // Test 1: Verify zero error for collinear points
    Point2 p1(0.0, 0.0);
    Point2 p2(2.0, 0.0);
    Point2 p3(4.0, 0.0);
    
    Vector error = factor.evaluateError(p1, p2, p3);
    EXPECT_NEAR(error(0), 0.0, 1e-10);
    
    // Test 2: Verify non-zero error when middle cone is off line
    Point2 p2_off(2.0, 1.0);
    Vector error_off = factor.evaluateError(p1, p2_off, p3);
    
    // Calculate expected error manually
    Vector2 v1 = p2_off - p1;  // (2, 1)
    Vector2 v2 = p3 - p1;      // (4, 0)
    double cross = v1.x() * v2.y() - v1.y() * v2.x();  // 2*0 - 1*4 = -4
    double norm = v1.norm() * v2.norm();  // sqrt(5) * 4
    double expected = cross / norm;
    
    EXPECT_NEAR(error_off(0), expected, 1e-10);
    
    // Test 3: Simple optimization test
    // We'll just test that the factor can be used in optimization
    NonlinearFactorGraph graph;
    Values initial;
    
    initial.insert(cone1, p1);
    initial.insert(cone2, p2_off);  // Off line
    initial.insert(cone3, p3);
    
    // Add strong priors on all points to make problem well-defined
    SharedNoiseModel tight_prior = noiseModel::Diagonal::Sigmas(Vector2(0.001, 0.001));
    graph.addPrior(cone1, p1, tight_prior);
    graph.addPrior(cone3, p3, tight_prior);
    
    // Add a weaker prior on middle point
    SharedNoiseModel weak_prior = noiseModel::Diagonal::Sigmas(Vector2(1.0, 1.0));
    graph.addPrior(cone2, Point2(2.0, 0.0), weak_prior);
    
    // Add the line factor
    graph.add(factor);
    
    // Just verify we can create and evaluate the graph
    double initial_error = graph.error(initial);
    EXPECT_GT(initial_error, 0.0);
    
    // Test passes if we get here without crashing
}

TEST_F(InterLandmarkFactorsTest, ConeAngleFactor) {
    // Test angle factor between three cones
    NonlinearFactorGraph graph;
    Values initial;
    
    // Create three cones forming a 90-degree angle
    Symbol cone1('L', 1);
    Symbol cone2('L', 2);  // vertex
    Symbol cone3('L', 3);
    
    initial.insert(cone1, Point2(0.0, 0.0));
    initial.insert(cone2, Point2(1.0, 0.0));
    initial.insert(cone3, Point2(1.0, 1.0));
    
    // Add angle factor for 90 degrees (pi/2 radians)
    double expected_angle = M_PI / 2.0;
    graph.add(ConeAngleFactor(cone1, cone2, cone3, expected_angle, angle_noise_));
    
    // Add distance factors to constrain the problem
    graph.add(ConeDistanceFactor(cone1, cone2, 1.0, distance_noise_));
    graph.add(ConeDistanceFactor(cone2, cone3, 1.0, distance_noise_));
    
    // Optimize
    LevenbergMarquardtParams params;
    LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    Values result = optimizer.optimize();
    
    // Check angle
    Point2 p1 = result.at<Point2>(cone1);
    Point2 p2 = result.at<Point2>(cone2);
    Point2 p3 = result.at<Point2>(cone3);
    
    Vector2 v1 = p1 - p2;
    Vector2 v2 = p3 - p2;
    double actual_angle = std::acos(v1.normalized().dot(v2.normalized()));
    
    EXPECT_NEAR(actual_angle, expected_angle, 0.01);
}

TEST_F(InterLandmarkFactorsTest, CompleteTrackScenario) {
    // Test a complete scenario with track boundaries
    NonlinearFactorGraph graph;
    Values initial;
    
    // Create a simple straight track section with parallel boundaries
    // Left boundary: L1, L2, L3
    // Right boundary: R1, R2, R3
    std::vector<Symbol> left_cones = {Symbol('L', 1), Symbol('L', 2), Symbol('L', 3)};
    std::vector<Symbol> right_cones = {Symbol('R', 1), Symbol('R', 2), Symbol('R', 3)};
    
    // Initialize positions
    for (size_t i = 0; i < 3; ++i) {
        initial.insert(left_cones[i], Point2(i * 2.0, 2.0));
        initial.insert(right_cones[i], Point2(i * 2.0, -2.0));
    }
    
    // Add collinearity constraints for boundaries
    graph.add(ConeLineFactor(left_cones[0], left_cones[1], left_cones[2], line_noise_));
    graph.add(ConeLineFactor(right_cones[0], right_cones[1], right_cones[2], line_noise_));
    
    // Add parallel lines factor
    graph.add(ConeParallelLinesFactor(
        left_cones[0], left_cones[1], 
        right_cones[0], right_cones[1], 
        line_noise_));
    
    // Add distance constraints along boundaries
    for (size_t i = 0; i < 2; ++i) {
        graph.add(ConeDistanceFactor(left_cones[i], left_cones[i+1], 2.0, distance_noise_));
        graph.add(ConeDistanceFactor(right_cones[i], right_cones[i+1], 2.0, distance_noise_));
    }
    
    // Optimize
    LevenbergMarquardtParams params;
    LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    Values result = optimizer.optimize();
    
    // Verify parallel boundaries
    Point2 l1 = result.at<Point2>(left_cones[0]);
    Point2 l2 = result.at<Point2>(left_cones[1]);
    Point2 r1 = result.at<Point2>(right_cones[0]);
    Point2 r2 = result.at<Point2>(right_cones[1]);
    
    Vector2 left_dir = (l2 - l1).normalized();
    Vector2 right_dir = (r2 - r1).normalized();
    
    // Directions should be parallel (dot product near 1 or -1)
    double dot_product = std::abs(left_dir.dot(right_dir));
    EXPECT_NEAR(dot_product, 1.0, 0.01);
    
    // Track width should be consistent
    double width1 = (l1 - r1).norm();
    double width2 = (l2 - r2).norm();
    EXPECT_NEAR(width1, width2, 0.01);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}