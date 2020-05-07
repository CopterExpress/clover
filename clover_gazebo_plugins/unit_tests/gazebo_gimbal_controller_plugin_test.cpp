#include <gtest/gtest.h>

#include <ignition/math.hh>

/* These declarations are needed here because the functions are not
 * public API of the class and therefore not declared in the header
 * of the plugin. */
namespace detail {
ignition::math::Vector3d ThreeAxisRot(
  double r11, double r12, double r21, double r31, double r32);

double NormalizeAbout(double _angle, double reference);

double ShortestAngularDistance(double _from, double _to);

ignition::math::Vector3d QtoZXY(
  const ignition::math::Quaterniond &_q);
}


////////////////////////////////////////////
/// ThreeAxisRot ///////////////////////////
////////////////////////////////////////////

TEST(ThreeAxisRot, Test1) {
    const auto res = detail::ThreeAxisRot(1.0, 2.0, 1.0, 3.0, 2.0);
    const ignition::math::Vector3d expected{0.982794, 1.5708, 0.463648};
    ASSERT_EQ(res, expected);
}

TEST(ThreeAxisRot, Test2) {
    const auto res = detail::ThreeAxisRot(0.6189, 0.3900, 0.4191, 0.9688, 0.2693);
    const ignition::math::Vector3d expected{1.29967, 0.432454, 1.0085};
    ASSERT_EQ(res, expected);
}

TEST(ThreeAxisRot, Test3) {
    const auto res = detail::ThreeAxisRot(0.1451, 0.4125, 0.2537, 0.0999, 0.1451);
    const ignition::math::Vector3d expected{0.60296, 0.256503, 0.33824};
    ASSERT_EQ(res, expected);
}

TEST(ThreeAxisRot, Test4) {
    const auto res = detail::ThreeAxisRot(0.7505, 0.1456, 0.8778, 0.5016, 0.4882);
    const ignition::math::Vector3d expected{0.798935, 1.07125, 1.37917};
    ASSERT_EQ(res, expected);
}

TEST(ThreeAxisRot, Test5) {
    const auto res = detail::ThreeAxisRot(-0.2354, 0.7973, -0.5751, -0.9311, 0.7262);
    const ignition::math::Vector3d expected{-0.908409, -0.612726, -0.28709};
    ASSERT_EQ(res, expected);
}

TEST(ThreeAxisRot, Test6) {
    const auto res = detail::ThreeAxisRot(-0.7793, 1.3942, 0.5090, -0.5165, -0.1568);
    const ignition::math::Vector3d expected{-1.86554, 0.534023, -0.509695};
    ASSERT_EQ(res, expected);
}

TEST(ThreeAxisRot, Test7) {
    const auto res = detail::ThreeAxisRot(1.7046, 1.8966, 0.9636, 1.3846, 0.5914);
    const ignition::math::Vector3d expected{1.16712, 1.30016, 0.732133};
    ASSERT_EQ(res, expected);
}


////////////////////////////////////////////
/// NormalizeAbout /////////////////////////
////////////////////////////////////////////

TEST(NormalizeAbout, Test1) {
    const auto res = detail::NormalizeAbout(1.0, 2.0);
    ASSERT_DOUBLE_EQ(res, 1.0);
}

TEST(NormalizeAbout, Test2) {
    const auto res = detail::NormalizeAbout(0.987, 1.234);
    ASSERT_DOUBLE_EQ(res, 0.987);
}

TEST(NormalizeAbout, Test3) {
    const auto res = detail::NormalizeAbout(-2.9776, 1.0559);
    ASSERT_DOUBLE_EQ(res, 3.3055853071795864);
}

TEST(NormalizeAbout, Test4) {
    const auto res = detail::NormalizeAbout(1.0723, 0.4975);
    ASSERT_DOUBLE_EQ(res, 1.0723);
}

TEST(NormalizeAbout, Test5) {
    const auto res = detail::NormalizeAbout(0.7538, 2.3674);
    ASSERT_DOUBLE_EQ(res, 0.7538);
}

TEST(NormalizeAbout, Test6) {
    const auto res = detail::NormalizeAbout(0.6982, -0.1008);
    ASSERT_DOUBLE_EQ(res, 0.6982);
}

TEST(NormalizeAbout, Test7) {
    const auto res = detail::NormalizeAbout(-0.948, 0.5941);
    ASSERT_DOUBLE_EQ(res, -0.948);
}


////////////////////////////////////////////
/// ShortestAngularDistance ////////////////
////////////////////////////////////////////

TEST(ShortestAngularDistance, Test1) {
    const auto res = detail::ShortestAngularDistance(0.987, 1.234);
    ASSERT_DOUBLE_EQ(res, 0.247);
}

TEST(ShortestAngularDistance, Test2) {
    const auto res = detail::ShortestAngularDistance(0.2782, 2.4551);
    ASSERT_DOUBLE_EQ(res, 2.1768999999999998);
}


TEST(ShortestAngularDistance, Test3) {
    const auto res = detail::ShortestAngularDistance(-0.1720, 0.0784);
    ASSERT_DOUBLE_EQ(res, 0.2504);
}

TEST(ShortestAngularDistance, Test4) {
    const auto res = detail::ShortestAngularDistance(1.3559, 0.8788);
    ASSERT_DOUBLE_EQ(res, -0.47710000000000008);
}

TEST(ShortestAngularDistance, Test5) {
    const auto res = detail::ShortestAngularDistance(-2.3618, 0.2399);
    ASSERT_DOUBLE_EQ(res, 2.6017000000000001);
}

TEST(ShortestAngularDistance, Test6) {
    const auto res = detail::ShortestAngularDistance(-0.8028, 2.1905);
    ASSERT_DOUBLE_EQ(res, 2.9933000000000001);
}

TEST(ShortestAngularDistance, Test7) {
    const auto res = detail::ShortestAngularDistance(0.8043, 0.4775);
    ASSERT_DOUBLE_EQ(res, -0.3268);
}


////////////////////////////////////////////
/// QtoZXY /////////////////////////////////
////////////////////////////////////////////

TEST(QtoZXY, Test1) {
    const auto res = detail::QtoZXY(ignition::math::Quaterniond());
    ASSERT_EQ(res, ignition::math::Vector3d());
}

TEST(QtoZXY, Test2) {
    const auto res = detail::QtoZXY(ignition::math::Quaterniond(0.45, 0.101, 0.138, 0.209));
    ASSERT_EQ(res, ignition::math::Vector3d(0.361318, 0.149136, 0.762717));
}

TEST(QtoZXY, Test3) {
    const auto res = detail::QtoZXY(ignition::math::Quaterniond(0.7774, -0.8116, 0.9980, 0.5101));
    ASSERT_EQ(res, ignition::math::Vector3d(1.89138, -0.246196, 1.29556));
}

TEST(QtoZXY, Test4) {
    const auto res = detail::QtoZXY(ignition::math::Quaterniond(0.2841, -0.8161, 0.1606, 2.2308));
    ASSERT_EQ(res, ignition::math::Vector3d(0.707386, 0.255599, 2.87201));
}

TEST(QtoZXY, Test5) {
    const auto res = detail::QtoZXY(ignition::math::Quaterniond(0.1764, -0.1524, -0.6796, 0.1968));
    ASSERT_EQ(res, ignition::math::Vector3d(-2.733, -0.327057, -0.309252));
}

TEST(QtoZXY, Test6) {
    const auto res = detail::QtoZXY(ignition::math::Quaterniond(0.1435, -0.1255, -0.6824, 0.3451));
    ASSERT_EQ(res, ignition::math::Vector3d(-2.83222, -0.531713, -0.202739));
}

TEST(QtoZXY, Test7) {
    const auto res = detail::QtoZXY(ignition::math::Quaterniond(0.0890, 0.2197, 0.6255, 0.4758));
    ASSERT_EQ(res, ignition::math::Vector3d(-2.69714, 0.687145, -0.991033));
}
