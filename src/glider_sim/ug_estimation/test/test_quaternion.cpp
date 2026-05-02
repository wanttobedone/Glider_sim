// 四元数工具单测：验证 Exp/Log 对偶性、注入 δθ 的小角度等价性。

#include <gtest/gtest.h>
#include "ug_estimation/core/quaternion_utils.h"

using namespace ug_ekf;

TEST(QuaternionUtils, ExpLogIdentity) {
  for (Scalar s : {Scalar(0), Scalar(1e-3), Scalar(0.1), Scalar(1.0), Scalar(2.5)}) {
    Vec3 phi(Scalar(0.3), Scalar(-0.7), Scalar(0.4));
    phi = phi.normalized() * s;
    Quat q = ExpQ(phi);
    Vec3 phi_back = LogQ(q);
    EXPECT_NEAR((phi - phi_back).norm(), 0.0, 1e-6) << "s=" << s;
  }
}

TEST(QuaternionUtils, IdentityRotation) {
  Quat q = ExpQ(Vec3::Zero());
  EXPECT_NEAR(q.w(), 1.0, 1e-12);
  EXPECT_NEAR(q.vec().norm(), 0.0, 1e-12);
}

TEST(QuaternionUtils, HatAntiSymmetric) {
  Vec3 v(1, 2, 3);
  Mat3 m = Hat(v);
  EXPECT_NEAR((m + m.transpose()).norm(), 0.0, 1e-12);
}

TEST(QuaternionUtils, InjectDeltaTheta_SmallAngle) {
  // 注入小角度 δθ 后旋转 ≈ ExpQ(δθ)·R
  Quat q0 = Quat::Identity();
  Vec3 dth(0.001, -0.002, 0.0015);
  Quat q1 = InjectDeltaTheta(q0, dth);

  Vec3 v(1, 0, 0);
  Vec3 v_rot = q1 * v;
  Vec3 v_expected = ExpQ(dth) * v;
  EXPECT_NEAR((v_rot - v_expected).norm(), 0.0, 1e-9);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
