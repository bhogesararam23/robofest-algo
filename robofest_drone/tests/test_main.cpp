#include "test_framework.h"
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "================================================================\n";
    std::cout << "  ROBOFEST GUJARAT 6.0 MINEFIELD SWARM DRONE - OFFLINE UNIT TESTS\n";
    std::cout << "================================================================\n";

    const auto& test_cases = TestFramework::get_registry();
    int passed = 0;
    int failed = 0;

    for (const auto& tc : test_cases) {
        std::cout << "[ RUN      ] " << tc.suite_name << "." << tc.test_name << " ... ";
        try {
            tc.func();
            std::cout << "[ PASS ]\n";
            passed++;
        } catch (const std::exception& ex) {
            std::cout << "[ FAIL ]\n";
            std::cout << "             Error: " << ex.what() << "\n";
            failed++;
        } catch (...) {
            std::cout << "[ FAIL ] (Unknown exception)\n";
            failed++;
        }
    }

    std::cout << "================================================================\n";
    std::cout << "  SUMMARY: Total: " << test_cases.size()
              << " | Passed: " << passed
              << " | Failed: " << failed << "\n";
    std::cout << "================================================================\n";

    return (failed == 0) ? 0 : 1;
}
