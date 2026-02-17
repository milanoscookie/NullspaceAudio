// Minimal test harness — no external dependencies
#pragma once
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

struct TestResult {
  std::string name;
  bool passed;
  std::string msg;
};

static std::vector<TestResult> g_results;
static int g_fails = 0;

#define TEST(name)                                                             \
  static void test_##name();                                                   \
  struct Register_##name {                                                     \
    Register_##name() { g_results.push_back({#name, true, ""}); }             \
  } reg_##name;                                                               \
  static void test_##name()

#define RUN_TEST(test_name)                                                    \
  do {                                                                         \
    try {                                                                      \
      test_##test_name();                                                      \
    } catch (const std::exception &e) {                                        \
      for (auto &tr_ : g_results)                                              \
        if (tr_.name == #test_name) {                                          \
          tr_.passed = false;                                                  \
          tr_.msg = std::string("EXCEPTION: ") + e.what();                     \
        }                                                                      \
      g_fails++;                                                               \
    }                                                                          \
  } while (0)

#define ASSERT_TRUE(cond)                                                      \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::cerr << "  FAIL: " << __FILE__ << ":" << __LINE__ << "  "           \
                << #cond << std::endl;                                         \
      g_fails++;                                                               \
      return;                                                                  \
    }                                                                          \
  } while (0)

#define ASSERT_EQ(a, b)                                                        \
  do {                                                                         \
    if ((a) != (b)) {                                                          \
      std::cerr << "  FAIL: " << __FILE__ << ":" << __LINE__ << "  "           \
                << #a << " == " << #b << "  (" << (a) << " vs " << (b)        \
                << ")" << std::endl;                                           \
      g_fails++;                                                               \
      return;                                                                  \
    }                                                                          \
  } while (0)

#define ASSERT_NEAR(a, b, tol)                                                 \
  do {                                                                         \
    if (std::abs((a) - (b)) > (tol)) {                                         \
      std::cerr << "  FAIL: " << __FILE__ << ":" << __LINE__ << "  "           \
                << #a << " ≈ " << #b << "  (" << (a) << " vs " << (b)        \
                << ", tol=" << (tol) << ")" << std::endl;                      \
      g_fails++;                                                               \
      return;                                                                  \
    }                                                                          \
  } while (0)

#define PRINT_RESULTS()                                                        \
  do {                                                                         \
    std::cout << "\n--- Results ---" << std::endl;                             \
    for (auto &r : g_results)                                                  \
      std::cout << (r.passed ? "  ✓ " : "  ✗ ") << r.name                    \
                << (r.msg.empty() ? "" : "  " + r.msg) << std::endl;          \
    std::cout << "\n" << g_results.size() << " tests, " << g_fails            \
              << " failures" << std::endl;                                     \
  } while (0)
