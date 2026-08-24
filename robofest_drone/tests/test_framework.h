#pragma once

#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <functional>
#include <stdexcept>

namespace TestFramework {

struct TestCase {
    std::string suite_name;
    std::string test_name;
    std::function<void()> func;
};

inline std::vector<TestCase>& get_registry() {
    static std::vector<TestCase> registry;
    return registry;
}

inline bool register_test(const std::string& suite, const std::string& name, std::function<void()> func) {
    get_registry().push_back({suite, name, func});
    return true;
}

#define TEST(suite, name) \
    void test_##suite##_##name(); \
    static bool reg_##suite##_##name = TestFramework::register_test(#suite, #name, test_##suite##_##name); \
    void test_##suite##_##name()

#define ASSERT_TRUE(cond) \
    if (!(cond)) { \
        throw std::runtime_error(std::string("Assertion failed: ") + #cond + " at " + __FILE__ + ":" + std::to_string(__LINE__)); \
    }

#define ASSERT_FALSE(cond) \
    if (cond) { \
        throw std::runtime_error(std::string("Assertion failed: !") + #cond + " at " + __FILE__ + ":" + std::to_string(__LINE__)); \
    }

#define ASSERT_EQ(a, b) \
    if ((a) != (b)) { \
        throw std::runtime_error(std::string("Assertion failed: ") + #a + " == " + #b + " at " + __FILE__ + ":" + std::to_string(__LINE__)); \
    }

#define ASSERT_FLOAT_EQ(a, b) \
    if (std::fabs((a) - (b)) > 0.005f) { \
        throw std::runtime_error(std::string("Assertion failed: ") + #a + " ~= " + #b + " at " + __FILE__ + ":" + std::to_string(__LINE__)); \
    }

#define ASSERT_NEAR(a, b, eps) \
    if (std::fabs((a) - (b)) > (eps)) { \
        throw std::runtime_error(std::string("Assertion failed: ") + #a + " near " + #b + " at " + __FILE__ + ":" + std::to_string(__LINE__)); \
    }

#define ASSERT_NE(a, b) \
    if ((a) == (b)) { \
        throw std::runtime_error(std::string("Assertion failed: ") + #a + " != " + #b + " at " + __FILE__ + ":" + std::to_string(__LINE__)); \
    }

} // namespace TestFramework
