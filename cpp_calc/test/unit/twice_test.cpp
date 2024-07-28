#include <cpp_calc/twice.hpp>
#include <gtest/gtest.h>

TEST(do_twice, two_should_be_four) {
  ASSERT_EQ(cpp_calc::do_twice(2), 4);
}
