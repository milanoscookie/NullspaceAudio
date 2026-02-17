// Tests for DoubleBufferSPSC<T>
#include "test_harness.h"
#include "utils/DoubleBufferSPSC.h"
#include <Eigen/Dense>

TEST(initial_tryRead_returns_false) {
  DoubleBufferSPSC<int> db;
  int out = -1;
  ASSERT_TRUE(!db.tryRead(out));
  ASSERT_EQ(out, -1);
}

TEST(publish_then_tryRead) {
  DoubleBufferSPSC<int> db;
  db.publish(42);
  int out = 0;
  ASSERT_TRUE(db.tryRead(out));
  ASSERT_EQ(out, 42);
}

TEST(tryRead_returns_false_if_no_new_data) {
  DoubleBufferSPSC<int> db;
  db.publish(1);
  int out = 0;
  db.tryRead(out); // consumes
  ASSERT_TRUE(!db.tryRead(out)); // nothing new
}

TEST(readLatest_always_returns_value) {
  DoubleBufferSPSC<int> db;
  db.publish(10);
  int out = 0;
  db.readLatest(out);
  ASSERT_EQ(out, 10);
  // Read again — same value
  db.readLatest(out);
  ASSERT_EQ(out, 10);
}

TEST(hasNew) {
  DoubleBufferSPSC<int> db;
  ASSERT_TRUE(!db.hasNew());
  db.publish(5);
  ASSERT_TRUE(db.hasNew());
  int out;
  db.tryRead(out);
  ASSERT_TRUE(!db.hasNew());
}

TEST(sequence_increments) {
  DoubleBufferSPSC<int> db;
  ASSERT_EQ(db.sequence(), 0u);
  db.publish(1);
  ASSERT_EQ(db.sequence(), 1u);
  db.publish(2);
  ASSERT_EQ(db.sequence(), 2u);
}

TEST(multiple_publishes_latest_wins) {
  DoubleBufferSPSC<int> db;
  db.publish(1);
  db.publish(2);
  db.publish(3);
  int out = 0;
  db.tryRead(out);
  ASSERT_EQ(out, 3);
}

TEST(beginWrite_commit) {
  DoubleBufferSPSC<int> db;
  int &ref = db.beginWrite();
  ref = 99;
  db.commit();
  int out = 0;
  ASSERT_TRUE(db.tryRead(out));
  ASSERT_EQ(out, 99);
}

TEST(works_with_eigen_block) {
  using Block = Eigen::Matrix<float, 4, 1>;
  DoubleBufferSPSC<Block> db;
  Block b;
  b << 1.0f, 2.0f, 3.0f, 4.0f;
  db.publish(b);
  Block out = Block::Zero();
  ASSERT_TRUE(db.tryRead(out));
  ASSERT_NEAR(out(0), 1.0f, 1e-6f);
  ASSERT_NEAR(out(3), 4.0f, 1e-6f);
}

int main() {
  RUN_TEST(initial_tryRead_returns_false);
  RUN_TEST(publish_then_tryRead);
  RUN_TEST(tryRead_returns_false_if_no_new_data);
  RUN_TEST(readLatest_always_returns_value);
  RUN_TEST(hasNew);
  RUN_TEST(sequence_increments);
  RUN_TEST(multiple_publishes_latest_wins);
  RUN_TEST(beginWrite_commit);
  RUN_TEST(works_with_eigen_block);
  PRINT_RESULTS();
  return g_fails > 0 ? 1 : 0;
}
