#include <cmath>
#include <chrono>

#include <ros2_lib/adder.h>

#include <gtest/gtest.h>

using namespace std;


/* TEST(TESTSuite, input_nan_checks) //{ */

TEST(TESTSuite, input_nan_checks) {

  int result = 1;

  if (ros2_lib::adder(1, 2) != 3) {
    result = 0;
  }

  EXPECT_TRUE(result);
}

//}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {

  testing::InitGoogleTest(&argc, argv);

  srand(static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count()));

  return RUN_ALL_TESTS();
}
