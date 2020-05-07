#include <gtest/gtest.h>

#include "gazebo_gps_plugin.h"


/* Wrap GpsPlugin to get access to protected functions
 * for testing. */
class GpsPluginTestHelper : public gazebo::GpsPlugin {
public:
    std::pair<double, double> reprojectWrapper(ignition::math::Vector3d& pos) {
        return gazebo::GpsPlugin::reproject(pos);
    }
};

#define CHECK_RESULT(a, b) do {SCOPED_TRACE("CHECK_RESULT"); checkResult((a),(b));}while(0)
void checkResult(std::pair<double, double> actual, std::pair<double, double> expected) {

    ASSERT_DOUBLE_EQ(actual.first, expected.first);
    ASSERT_DOUBLE_EQ(actual.second, expected.second);
}

TEST(GPS, TestReproject1) {
    GpsPluginTestHelper testHelper;

    ignition::math::Vector3d pos{0.982794, 1.5708, 0.463648};

    std::pair<double, double> expected{0.82724690316405536, 0.14914898037542368};

    auto const actual = testHelper.reprojectWrapper(pos);

    CHECK_RESULT(actual, expected);
}


TEST(GPS, TestReproject2) {
    GpsPluginTestHelper testHelper;

    ignition::math::Vector3d pos{0.7833148, 0.1725992, 0.8749541};

    std::pair<double, double> expected{0.8272466830789349, 0.14914893398892642};
    auto const actual = testHelper.reprojectWrapper(pos);

    CHECK_RESULT(actual, expected);
}


TEST(GPS, TestReproject3) {
    GpsPluginTestHelper testHelper;

    ignition::math::Vector3d pos{0.82724671377633008, 0.3676197, 0.5845087};

    std::pair<double, double> expected{0.82724671377632109, 0.14914894420476085};

    auto const actual = testHelper.reprojectWrapper(pos);

    CHECK_RESULT(actual, expected);
}

TEST(GPS, TestReproject4) {
    GpsPluginTestHelper testHelper;

    ignition::math::Vector3d pos{3.0448264, 2.3968121, 0.2209594};

    std::pair<double, double> expected{0.82724703318316262, 0.14914945987599484};

    auto const actual = testHelper.reprojectWrapper(pos);

    CHECK_RESULT(actual, expected);
}

TEST(GPS, TestReproject5) {
    GpsPluginTestHelper testHelper;

    ignition::math::Vector3d pos{3.017489057, 0.8586486, 0.3064644};

    std::pair<double, double> expected{0.8272467910670781, 0.14914945351884371};

    auto const actual = testHelper.reprojectWrapper(pos);

    CHECK_RESULT(actual, expected);
}

TEST(GPS, TestReproject6) {
    GpsPluginTestHelper testHelper;

    ignition::math::Vector3d pos{0.5935629, 3.0111186, 0.8701562};

    std::pair<double, double> expected{0.82724712987878102, 0.1491488898645153};

    auto const actual = testHelper.reprojectWrapper(pos);

    CHECK_RESULT(actual, expected);
}

TEST(GPS, TestReproject7) {
    GpsPluginTestHelper testHelper;

    ignition::math::Vector3d pos{0.0658477, 2.7016995, 0.4642607};

    std::pair<double, double> expected{0.82724708117437828, 0.14914876715075434};

    auto const actual = testHelper.reprojectWrapper(pos);

    CHECK_RESULT(actual, expected);
}
