#include <gtest/gtest.h>
#include <cmath>
#include "crowd_sim/planning_math.hpp"

namespace {

// Mirrors config/controller_params.yaml with angles in radians.
PredictParams default_params() {
  return PredictParams{
    .ahead_time = 1.0,
    .time_resolution = 0.2,
    .min_speed = 0.2,
    .max_speed = 1.5,
    .preferred_speed = 1.2,
    .speed_resolution = 0.1,
    .max_angular_vel = DEG2RAD(40.0),
    .angular_vel_resolution = DEG2RAD(5.0),
    .dist_cost = 2.,
    .away_from_goal_cost = 0.2,
    .diff_from_prev_output_cost = 0.05,
    .diff_from_preferred_speed_cost = 0.1,
  };
}

constexpr double kTol = 1e-9;

}  // namespace

// Grid-size macros. With the default params the intended grids are:
//   times: 0.0, 0.2, ..., 1.0            -> 6 samples
//   speeds: 0.2, 0.3, ..., 1.5           -> 14 samples
//   angular vels: -40, -35, ..., 40 deg  -> 17 samples (odd, so 0 is included)
TEST(GridMacros, NumTimes) {
  PredictParams p = default_params();
  EXPECT_EQ(NUM_TIMES(p), 6);
}

TEST(GridMacros, NumSpeeds) {
  PredictParams p = default_params();
  EXPECT_EQ(NUM_SPEEDS(p), 14);
}

TEST(GridMacros, NumAngularVels) {
  PredictParams p = default_params();
  EXPECT_EQ(NUM_ANGULAR_VELS(p), 17);
  EXPECT_EQ(NUM_ANGULAR_VELS(p) % 2, 1);
}

TEST(GridMacros, NumTimesWithExactDivision) {
  PredictParams p = default_params();
  p.ahead_time = 2.0;
  p.time_resolution = 0.5;
  EXPECT_EQ(NUM_TIMES(p), 5);
}

TEST(GridMacros, IdxToTime) {
  PredictParams p = default_params();
  EXPECT_NEAR(IDX_TO_TIME(p, 0), 0.0, kTol);
  EXPECT_NEAR(IDX_TO_TIME(p, 3), 0.6, kTol);
  EXPECT_NEAR(IDX_TO_TIME(p, NUM_TIMES(p) - 1), p.ahead_time, 1e-9);
}

TEST(GridMacros, IdxToSpeed) {
  PredictParams p = default_params();
  EXPECT_NEAR(IDX_TO_SPEED(p, 0), p.min_speed, kTol);
  EXPECT_NEAR(IDX_TO_SPEED(p, 1), 0.3, kTol);
  EXPECT_NEAR(IDX_TO_SPEED(p, NUM_SPEEDS(p) - 1), p.max_speed, 1e-9);
}

TEST(GridMacros, IdxToAngularVel) {
  PredictParams p = default_params();
  int n = NUM_ANGULAR_VELS(p);
  // First and last indices hit the extremes, the middle index is exactly 0.
  EXPECT_NEAR(IDX_TO_ANGULAR_VEL(p, 0), -p.max_angular_vel, 1e-9);
  EXPECT_NEAR(IDX_TO_ANGULAR_VEL(p, n - 1), p.max_angular_vel, 1e-9);
  EXPECT_NEAR(IDX_TO_ANGULAR_VEL(p, n / 2), 0.0, kTol);
  // Symmetric about the middle.
  EXPECT_NEAR(IDX_TO_ANGULAR_VEL(p, 2), -IDX_TO_ANGULAR_VEL(p, n - 3), kTol);
}

TEST(GridMacros, MacrosEvaluateArgumentsSafely) {
  // The idx argument is used more than once in some macros, so an expression
  // with a side effect must not be evaluated twice.
  PredictParams p = default_params();
  int i = 0;
  double t = IDX_TO_TIME(p, i++);
  EXPECT_EQ(i, 1);
  EXPECT_NEAR(t, 0.0, kTol);
  i = 0;
  double w = IDX_TO_ANGULAR_VEL(p, i++);
  EXPECT_EQ(i, 1);
  EXPECT_NEAR(w, -p.max_angular_vel, 1e-9);
}

TEST(AngleTraveled, EqualsAngVelTimesTime) {
  // angle = (v * t) / (v / w) = w * t
  OutputInfo o{.speed = 1.0, .ang_vel = 0.5, .time = 2.0};
  EXPECT_NEAR(calculate_angle_traveled(o), 1.0, kTol);
}

TEST(FuturePositions, ConstantVelocity) {
  vec2d_list pos{Vector2d(0, 0), Vector2d(1, 1)};
  vec2d_list vel{Vector2d(1, 0), Vector2d(0, -2)};
  vec2d_list out = get_future_positions(pos, vel, 0.5);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_NEAR(out[0].x(), 0.5, kTol);
  EXPECT_NEAR(out[0].y(), 0.0, kTol);
  EXPECT_NEAR(out[1].x(), 1.0, kTol);
  EXPECT_NEAR(out[1].y(), 0.0, kTol);
}

TEST(FutureRobotPosition, QuarterCircleLeftTurn) {
  // speed 1, ang_vel 1 -> radius 1. Quarter turn in pi/2 seconds.
  OutputInfo o{.speed = 1.0, .ang_vel = 1.0, .time = M_PI / 2};
  Vector2d p = get_future_robot_position(Vector2d(0, 0), 0.0, o);
  EXPECT_NEAR(p.x(), -1., 1e-9);
  EXPECT_NEAR(p.y(), 1.0, 1e-9);
}

TEST(FutureRobotPosition, HeadingIsApplied) {
  // Same arc, robot facing +y. The displacement should rotate with it.
  OutputInfo o{.speed = 1.0, .ang_vel = 1.0, .time = M_PI / 2};
  Vector2d p = get_future_robot_position(Vector2d(2, 3), M_PI / 2, o);
  EXPECT_NEAR(p.x(), 2. - 1., 1e-9);
  EXPECT_NEAR(p.y(), 3.0 - 1.0, 1e-9);
}

TEST(TotalSquaredDist, SumsOverAgents) {
  PredictParams params = default_params();
  vec2d_list pos{Vector2d(1, 0), Vector2d(0, 2)};
  EXPECT_NEAR(calculate_dist_cost(params, pos, Vector2d(0, 0)), 1.0 + 4.0, kTol);
  EXPECT_NEAR(calculate_dist_cost(params, {}, Vector2d(5, 5)), 0.0, kTol);
}

TEST(AwayFromGoalCost, ZeroWhenHeadingAtGoal) {
  PredictParams p = default_params();
  // Tiny turn so angle traveled is negligible but ang_vel is non-zero.
  OutputInfo o{.speed = 1.0, .ang_vel = 1e-6, .time = 1.0};
  double cost = calculate_away_from_goal_cost(p, Vector2d(-5.5, 0), 0.0, Vector2d(5, 0), o);
  EXPECT_NEAR(cost, 0.0, 1e-9);
}

TEST(AwayFromGoalCost, MaxWhenFacingAway) {
  PredictParams p = default_params();
  OutputInfo o{.speed = 1.0, .ang_vel = 1e-6, .time = 1.0};
  double cost = calculate_away_from_goal_cost(p, Vector2d(-5.5, 0), M_PI, Vector2d(5, 0), o);
  EXPECT_NEAR(cost, 1.0 * p.away_from_goal_cost, 1e-9);
}

TEST(AwayFromGoalCost, ScalesWithSpeed) {
  PredictParams p = default_params();
  OutputInfo slow{.speed = 0.5, .ang_vel = 1e-6, .time = 1.0};
  OutputInfo fast{.speed = 1.5, .ang_vel = 1e-6, .time = 1.0};
  double c_slow = calculate_away_from_goal_cost(p, Vector2d(0, 0), M_PI, Vector2d(5, 0), slow);
  double c_fast = calculate_away_from_goal_cost(p, Vector2d(0, 0), M_PI, Vector2d(5, 0), fast);
  EXPECT_NEAR(c_fast / c_slow, 3.0, 1e-9);
}

TEST(AwayFromGoalCost, QuadraticInMisalignment) {
  PredictParams p = default_params();
  OutputInfo o{.speed = 1.0, .ang_vel = 1e-6, .time = 1.0};
  // Heading perpendicular to the goal direction: dot = 0, so the linear
  // misalignment term is 0.5 and its square is 0.25.
  double perp = calculate_away_from_goal_cost(p, Vector2d(0, 0), M_PI / 2, Vector2d(5, 0), o);
  EXPECT_NEAR(perp, 0.25 * p.away_from_goal_cost, 1e-9);

  // 60 degrees off: dot = 0.5, linear term 0.25, squared 0.0625.
  double sixty = calculate_away_from_goal_cost(p, Vector2d(0, 0), M_PI / 3, Vector2d(5, 0), o);
  EXPECT_NEAR(sixty, 0.0625 * p.away_from_goal_cost, 1e-9);

  // Quadratic: doubling the linear term quadruples the cost.
  EXPECT_NEAR(perp / sixty, 4.0, 1e-9);
}

TEST(DiffFromPrevOutputCost, ZeroForIdenticalCommands) {
  PredictParams p = default_params();
  OutputInfo o{.speed = 1.0, .ang_vel = DEG2RAD(5.0), .time = 0.5};
  EXPECT_NEAR(calculate_diff_from_prev_output_cost(p, o, o), 0.0, kTol);
}

TEST(DiffFromPrevOutputCost, SpeedChangeSameDirection) {
  PredictParams p = default_params();
  OutputInfo cur{.speed = 1.5, .ang_vel = DEG2RAD(5.0), .time = 0.5};
  OutputInfo prev{.speed = 0.5, .ang_vel = DEG2RAD(5.0), .time = 0.5};
  // |1.5 - 0.5|^2 * 0.05
  EXPECT_NEAR(calculate_diff_from_prev_output_cost(p, cur, prev), 1.0 * 0.05, 1e-9);
}

TEST(DiffFromPrevOutputCost, SymmetricInArguments) {
  PredictParams p = default_params();
  OutputInfo a{.speed = 1.0, .ang_vel = DEG2RAD(40.0), .time = 1.0};
  OutputInfo b{.speed = 1.0, .ang_vel = -DEG2RAD(40.0), .time = 1.0};
  EXPECT_NEAR(calculate_diff_from_prev_output_cost(p, a, b),
              calculate_diff_from_prev_output_cost(p, b, a), kTol);
}

TEST(DiffFromPreferredSpeedCost, ZeroAtPreferredSpeed) {
  PredictParams p = default_params();
  EXPECT_NEAR(calculate_diff_from_preferred_speed_cost(p, p.preferred_speed), 0.0, kTol);
}

TEST(DiffFromPreferredSpeedCost, QuadraticInDifference) {
  PredictParams p = default_params();
  // |0.7 - 1.2|^2 * 0.1
  EXPECT_NEAR(calculate_diff_from_preferred_speed_cost(p, 0.7), 0.25 * 0.1, 1e-9);
  // Doubling the difference quadruples the cost.
  double c1 = calculate_diff_from_preferred_speed_cost(p, p.preferred_speed + 0.2);
  double c2 = calculate_diff_from_preferred_speed_cost(p, p.preferred_speed + 0.4);
  EXPECT_NEAR(c2 / c1, 4.0, 1e-9);
}

TEST(DiffFromPreferredSpeedCost, SymmetricAboveAndBelow) {
  PredictParams p = default_params();
  double above = calculate_diff_from_preferred_speed_cost(p, p.preferred_speed + 0.3);
  double below = calculate_diff_from_preferred_speed_cost(p, p.preferred_speed - 0.3);
  EXPECT_NEAR(above, below, kTol);
}

TEST(DiffFromPreferredSpeedCost, ScalesWithWeight) {
  PredictParams p = default_params();
  double base = calculate_diff_from_preferred_speed_cost(p, 0.5);
  p.diff_from_preferred_speed_cost *= 3.0;
  EXPECT_NEAR(calculate_diff_from_preferred_speed_cost(p, 0.5), 3.0 * base, 1e-9);
}

TEST(DiffFromPreferredSpeedCost, ZeroWeightDisablesCost) {
  PredictParams p = default_params();
  p.diff_from_preferred_speed_cost = 0.0;
  EXPECT_NEAR(calculate_diff_from_preferred_speed_cost(p, 0.2), 0.0, kTol);
}

TEST(ZeroAngularVelocity, AngleTraveledIsZero) {
  OutputInfo o{.speed = 1.0, .ang_vel = 0.0, .time = 0.5};
  EXPECT_NEAR(calculate_angle_traveled(o), 0.0, kTol);
}

TEST(ZeroAngularVelocity, FutureRobotPositionIsStraightLine) {
  OutputInfo o{.speed = 1.0, .ang_vel = 0.0, .time = 0.5};
  Vector2d p = get_future_robot_position(Vector2d(0, 0), 0.0, o);
  EXPECT_NEAR(p.x(), 0.5, kTol);
  EXPECT_NEAR(p.y(), 0.0, kTol);
}

TEST(ZeroAngularVelocity, DiffCostFromZeroPrevOutputIsFinite) {
  PredictParams p = default_params();
  OutputInfo cur{.speed = 1.0, .ang_vel = DEG2RAD(5.0), .time = 0.5};
  OutputInfo prev{.speed = 0.0, .ang_vel = 0.0, .time = 0.5};
  EXPECT_FALSE(std::isnan(calculate_diff_from_prev_output_cost(p, cur, prev)));
}
